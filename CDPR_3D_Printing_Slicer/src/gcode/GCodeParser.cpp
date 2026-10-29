#include "cdpr_slicer/Planner.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {

struct ImportError : std::runtime_error {
    DiagnosticCode code;
    ImportError(DiagnosticCode value, const std::string& message)
        : std::runtime_error(message), code(value) {}
};

void Require(bool condition, DiagnosticCode code, const std::string& message) {
    if(!condition) throw ImportError(code, message);
}

bool Space(char c) { return c == ' ' || c == '\t'; }
bool Digit(char c) { return c >= '0' && c <= '9'; }
bool Letter(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }
char Upper(char c) { return c >= 'a' && c <= 'z' ? static_cast<char>(c - 'a' + 'A') : c; }
std::string Trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t");
    return first == std::string::npos ? std::string() : text.substr(first, text.find_last_not_of(" \t") - first + 1);
}

// UTF-8 is permitted in comments/metadata, never in numeric command words.
void ValidateEncoding(const std::string& text) {
    for(std::size_t i = 0; i < text.size();){
        const auto a = static_cast<unsigned char>(text[i]);
        Require(a != 0, DiagnosticCode::malformed_input, "Embedded NUL is forbidden.");
        if(a < 0x80){
            Require((a >= 0x20 && a < 0x7F) || a == '\r' || a == '\n' || a == '\t', DiagnosticCode::malformed_input, "Unsupported control byte.");
            ++i;
            continue;
        }
        const std::size_t length = a >= 0xC2 && a <= 0xDF ? 2 : a >= 0xE0 && a <= 0xEF ? 3 : a >= 0xF0 && a <= 0xF4 ? 4 : 0;
        Require(length != 0 && length <= text.size() - i, DiagnosticCode::malformed_input, "Invalid UTF-8.");
        for(std::size_t j = 1; j < length; ++j){
            const auto b = static_cast<unsigned char>(text[i + j]);
            Require(b >= 0x80 && b <= 0xBF, DiagnosticCode::malformed_input, "Invalid UTF-8 continuation.");
        }
        const auto b = static_cast<unsigned char>(text[i + 1]);
        Require(!(a == 0xE0 && b < 0xA0) && !(a == 0xED && b >= 0xA0)
            && !(a == 0xF0 && b < 0x90) && !(a == 0xF4 && b > 0x8F), DiagnosticCode::malformed_input, "Invalid UTF-8 scalar.");
        i += length;
    }
}

double Decimal(const std::string& token) {
    std::size_t p = 0;
    if(p < token.size() && (token[p] == '+' || token[p] == '-')) ++p;
    bool digits = false;
    while(p < token.size() && Digit(token[p])) { digits = true; ++p; }
    if(p < token.size() && token[p] == '.'){
        ++p;
        while(p < token.size() && Digit(token[p])) { digits = true; ++p; }
    }
    Require(digits && p == token.size(), DiagnosticCode::invalid_number, "Expected a complete decimal number.");
    std::istringstream input(token);
    input.imbue(std::locale::classic());
    double result = 0;
    input >> result;
    Require(!input.fail() && input.peek() == std::char_traits<char>::eof() && std::isfinite(result), DiagnosticCode::invalid_number, "Decimal is out of range.");
    Require(result != 0 || std::none_of(token.begin(), token.end(), [](char c) { return c >= '1' && c <= '9'; }),
        DiagnosticCode::invalid_number, "Nonzero decimal underflow.");
    return result;
}

std::uint64_t Unsigned(const std::string& token) {
    Require(!token.empty(), DiagnosticCode::invalid_number, "Missing integer.");
    std::uint64_t value = 0;
    for(char c : token){
        Require(Digit(c), DiagnosticCode::invalid_number, "Expected an unsigned decimal integer.");
        const auto digit = static_cast<std::uint64_t>(c - '0');
        Require(value <= (9007199254740991ULL - digit) / 10, DiagnosticCode::invalid_number, "Integer exceeds the interchange range.");
        value = value * 10 + digit;
    }
    return value;
}

struct Word { char letter; std::string token; double value; };
Word ReadWord(const std::string& text, std::size_t& p) {
    while(p < text.size() && Space(text[p])) ++p;
    Require(p < text.size() && Letter(text[p]), DiagnosticCode::malformed_input, "Expected a command or parameter letter.");
    Word result;
    result.letter = Upper(text[p++]);
    while(p < text.size() && Space(text[p])) ++p;
    const auto start = p;
    if(p < text.size() && (text[p] == '+' || text[p] == '-')) ++p;
    while(p < text.size() && Digit(text[p])) ++p;
    if(p < text.size() && text[p] == '.'){
        ++p;
        while(p < text.size() && Digit(text[p])) ++p;
    }
    result.token = text.substr(start, p - start);
    result.value = Decimal(result.token);
    return result;
}

std::string StripComments(const std::string& text, std::string& comments) {
    std::string code;
    bool parenthesis = false;
    for(std::size_t i = 0; i < text.size(); ++i){
        const char c = text[i];
        if(c == '('){
            Require(!parenthesis, DiagnosticCode::malformed_input, "Nested comment.");
            parenthesis = true;
            code += ' ';
        }else if(c == ')'){
            Require(parenthesis, DiagnosticCode::malformed_input, "Unmatched closing comment.");
            parenthesis = false;
            comments += ' ';
        }else if(parenthesis){ comments += c; }
        else if(c == ';'){
            comments += text.substr(i + 1);
            break;
        }else{ code += c; }
    }
    Require(!parenthesis, DiagnosticCode::malformed_input, "Unterminated comment.");
    return Trim(code);
}

struct Line {
    bool has_command{false};
    char family{'G'};
    std::uint64_t command{0};
    std::map<char, double> parameters;
};

class Importer {
    const GCodeParseRequest& request_;
    PrintJob job_;
    SourceLocation location_;
    SourceLengthUnit units_;
    PositionMode xyz_mode_;
    PositionMode e_mode_;
    OptionalValue<CartesianPose> pose_;
    std::array<double, 3> xyz_origin_mm_{};
    OptionalValue<double> logical_e_mm_;
    OptionalValue<double> filament_mm_;
    OptionalValue<double> feed_mm_per_min_;
    OptionalValue<std::uint64_t> next_line_number_;
    OptionalValue<std::uint64_t> layer_;
    bool numbered_{false};
    bool checksum_mode_{false};
    bool checksum_mode_known_{false};
    bool motion_mode_{false};
    double retraction_debt_mm_{0};
    std::size_t physical_line_{0};

    double UnitMillimetres() const {
        Require(units_ != SourceLengthUnit::unspecified, DiagnosticCode::undefined_modal_state, "G20/G21 or explicit initial units are required.");
        return units_ == SourceLengthUnit::inches ? 25.4 : 1.0;
    }
    double Length(double value) const {
        const double converted = value * UnitMillimetres();
        Require(std::isfinite(converted), DiagnosticCode::invalid_number, "Unit conversion overflow.");
        return converted;
    }
    void Info(DiagnosticCode code, const std::string& message) {
        Require(job_.validation.diagnostics.size() < request_.options.maximum_diagnostics, DiagnosticCode::limit_exceeded, "Diagnostic limit exceeded.");
        Diagnostic d;
        d.severity = Severity::info;
        d.code = code;
        d.message = message;
        d.source.has_value = true;
        d.source.value = location_;
        job_.validation.diagnostics.push_back(std::move(d));
    }
    void Add(Operation op) {
        // Reserve space for the final end_job; never return a partial import.
        Require(job_.operations.size() + 1 < request_.options.maximum_operations, DiagnosticCode::limit_exceeded, "Operation limit exceeded.");
        op.id = static_cast<std::uint64_t>(job_.operations.size());
        op.source = location_;
        job_.operations.push_back(std::move(op));
    }
    static void Allowed(const Line& line, const std::string& letters) {
        for(const auto& parameter : line.parameters)
            Require(letters.find(parameter.first) != std::string::npos, DiagnosticCode::unsupported_command, "Unsupported parameter on this command.");
    }
    static bool Has(const Line& line, char c) { return line.parameters.count(c) != 0; }
    static double Get(const Line& line, char c) { return line.parameters.at(c); }
    static void ZeroChannel(const Line& line, char c) {
        Require(!Has(line, c) || Get(line, c) == 0.0, DiagnosticCode::unsupported_command, "Only channel/tool zero is supported.");
    }

    std::string VerifyEnvelope(const std::string& raw) {
        std::size_t star = std::string::npos;
        bool comment = false;
        for(std::size_t i = 0; i < raw.size(); ++i){
            if(raw[i] == ';' && !comment) break;
            if(raw[i] == '('){
                Require(!comment, DiagnosticCode::malformed_input, "Nested comment.");
                comment = true;
            }else if(raw[i] == ')'){
                Require(comment, DiagnosticCode::malformed_input, "Unmatched closing comment.");
                comment = false;
            }else if(raw[i] == '*' && !comment){ star = i; break; }
        }
        std::string body = raw;
        if(star != std::string::npos){
            const auto suffix = Trim(raw.substr(star + 1));
            const auto expected = Unsigned(suffix);
            Require(expected <= 255, DiagnosticCode::malformed_input, "Checksum is outside byte range.");
            unsigned char checksum = 0;
            for(std::size_t i = 0; i < star; ++i){
                Require(static_cast<unsigned char>(raw[i]) < 128, DiagnosticCode::malformed_input, "Checksummed lines must be ASCII.");
                checksum ^= static_cast<unsigned char>(raw[i]);
            }
            Require(checksum == expected, DiagnosticCode::identity_mismatch, "Line checksum mismatch.");
            body = raw.substr(0, star);
        }
        std::size_t p = 0;
        while(p < body.size() && Space(body[p])) ++p;
        const bool has_number = p < body.size() && Upper(body[p]) == 'N';
        if(has_number){
            const auto word = ReadWord(body, p);
            const auto number = Unsigned(word.token);
            if(next_line_number_.has_value)
                Require(number == next_line_number_.value, DiagnosticCode::malformed_input, "Numbered lines must be contiguous.");
            Require(number < 9007199254740991ULL, DiagnosticCode::limit_exceeded, "Line sequence exhausted.");
            next_line_number_.has_value = true;
            next_line_number_.value = number + 1;
            numbered_ = true;
            if(checksum_mode_known_)
                Require(checksum_mode_ == (star != std::string::npos), DiagnosticCode::malformed_input, "Do not mix checksummed and unchecked numbered lines.");
            checksum_mode_ = star != std::string::npos;
            checksum_mode_known_ = true;
            body = body.substr(p);
        }else{
            Require(star == std::string::npos, DiagnosticCode::malformed_input, "A checksum requires an N line number.");
        }
        std::string comments;
        const auto code = StripComments(body, comments);
        if(numbered_ && !has_number)
            Require(code.empty(), DiagnosticCode::malformed_input, "An executable line is missing its N number.");
        const auto marker = comments.find("LAYER:");
        if(marker != std::string::npos){
            const auto annotation = Trim(comments.substr(marker + 6));
            if(!annotation.empty() && std::all_of(annotation.begin(), annotation.end(), Digit)){
                layer_.has_value = true;
                layer_.value = Unsigned(annotation);
            }
        }
        return code;
    }

    Line ParseLine(const std::string& code) {
        Line result;
        std::size_t p = 0;
        while(p < code.size()){
            const auto word = ReadWord(code, p);
            const bool tool_parameter = word.letter == 'T' && result.has_command
                && result.family == 'M' && (result.command == 104 || result.command == 109);
            if(word.letter == 'G' || word.letter == 'M' || (word.letter == 'T' && !tool_parameter)){
                Require(!result.has_command && result.parameters.empty(), DiagnosticCode::malformed_input, "Only one leading command is allowed on a line.");
                result.has_command = true;
                result.family = word.letter;
                result.command = Unsigned(word.token);
                if(result.family == 'M' && result.command == 117){
                    break;
                }
            }else{
                Require(word.letter != 'N', DiagnosticCode::malformed_input, "N must prefix the line.");
                Require(result.parameters.emplace(word.letter, word.value).second, DiagnosticCode::malformed_input, "Duplicate parameter.");
            }
            while(p < code.size() && Space(code[p])) ++p;
        }
        return result;
    }

    void Move(const Line& line) {
        Allowed(line, "XYZEF");
        if(Has(line, 'F')){
            const double feed = Length(Get(line, 'F'));
            Require(feed > 0, DiagnosticCode::invalid_number, "Feedrate must be positive.");
            feed_mm_per_min_.has_value = true;
            feed_mm_per_min_.value = feed;
        }
        const bool xyz = Has(line, 'X') || Has(line, 'Y') || Has(line, 'Z');
        const bool extrusion = Has(line, 'E');
        if(!xyz && !extrusion){ Info(DiagnosticCode::state_change, "Modal feedrate/motion mode updated; no movement."); return; }
        Require(pose_.has_value, DiagnosticCode::undefined_modal_state, "An explicit starting nozzle pose is required.");
        auto end = pose_.value;
        if(xyz){
            Require(xyz_mode_ != PositionMode::unspecified, DiagnosticCode::undefined_modal_state, "XYZ positioning mode is undefined.");
            const char axes[3] = {'X', 'Y', 'Z'};
            for(std::size_t i = 0; i < 3; ++i) if(Has(line, axes[i])){
                const double value = Length(Get(line, axes[i]));
                Require(value == 0 || value / 1000.0 != 0, DiagnosticCode::invalid_number, "Position conversion underflow.");
                const double previous = end.position_m[i];
                end.position_m[i] = xyz_mode_ == PositionMode::relative
                    ? end.position_m[i] + value / 1000.0
                    : (xyz_origin_mm_[i] + value) / 1000.0;
                Require(std::isfinite(end.position_m[i]), DiagnosticCode::invalid_number, "Position arithmetic overflow.");
                Require(xyz_mode_ != PositionMode::relative || value == 0 || end.position_m[i] != previous,
                    DiagnosticCode::invalid_number, "Relative axis increment was lost to position precision.");
            }
        }
        double delta = 0;
        if(extrusion){
            Require(e_mode_ != PositionMode::unspecified && logical_e_mm_.has_value && filament_mm_.has_value,
                DiagnosticCode::undefined_modal_state, "E mode, logical E and a commanded-filament datum are required.");
            const double e = Length(Get(line, 'E'));
            delta = e_mode_ == PositionMode::relative ? e : e - logical_e_mm_.value;
            Require(std::isfinite(delta), DiagnosticCode::invalid_number, "Extrusion arithmetic overflow.");
        }
        const bool moved = end.position_m != pose_.value.position_m;
        if(!moved && delta == 0){ Info(DiagnosticCode::no_op, "No-op movement omitted."); return; }
        Require(feed_mm_per_min_.has_value, DiagnosticCode::undefined_modal_state, "Movement feedrate is undefined.");
        // Even travel records preserve known E accounting; never invent an E origin.
        Require(logical_e_mm_.has_value && filament_mm_.has_value, DiagnosticCode::undefined_modal_state, "Declare the logical E and commanded-filament datum before the first movement.");
        MotionOperation motion;
        motion.nozzle.frame_id = job_.frames.bed_frame_id;
        motion.nozzle.start = pose_.value;
        motion.nozzle.end = end;
        motion.delta_filament_mm = delta;
        motion.logical_e_start_mm = logical_e_mm_.value;
        motion.logical_e_end_mm = logical_e_mm_.value + delta;
        motion.filament_position_start_mm = filament_mm_.value;
        motion.filament_position_end_mm = filament_mm_.value + delta;
        Require(std::isfinite(motion.logical_e_end_mm) && std::isfinite(motion.filament_position_end_mm), DiagnosticCode::invalid_number, "Filament position overflow.");
        if(moved){
            motion.path_feedrate_m_per_s.has_value = true;
            motion.path_feedrate_m_per_s.value = feed_mm_per_min_.value / 60000.0;
            Require(motion.path_feedrate_m_per_s.value > 0, DiagnosticCode::invalid_number, "Path feedrate underflow.");
        }else{
            motion.filament_feedrate_mm_per_s.has_value = true;
            motion.filament_feedrate_mm_per_s.value = feed_mm_per_min_.value / 60.0;
            Require(motion.filament_feedrate_mm_per_s.value > 0, DiagnosticCode::invalid_number, "Filament feedrate underflow.");
        }
        ExtrusionSpan span;
        span.delta_filament_mm = delta;
        if(delta < 0){
            span.purpose = ExtrusionPurpose::retract;
            motion.intent = MotionIntent::retract;
            retraction_debt_mm_ -= delta;
        }else if(delta == 0){ motion.intent = MotionIntent::travel; }
        else{
            const double recovered = std::min(delta, retraction_debt_mm_);
            retraction_debt_mm_ -= recovered;
            if(recovered == delta){ span.purpose = ExtrusionPurpose::recover; motion.intent = MotionIntent::recover; }
            else{
                span.purpose = moved ? ExtrusionPurpose::deposit : ExtrusionPurpose::prime;
                motion.intent = moved ? MotionIntent::deposit : MotionIntent::prime;
                if(recovered > 0){
                    ExtrusionSpan recovery;
                    recovery.end_fraction = recovered / delta;
                    Require(recovery.end_fraction > 0 && recovery.end_fraction < 1, DiagnosticCode::invalid_number, "Extrusion span cannot be represented without losing its boundary.");
                    recovery.purpose = ExtrusionPurpose::recover;
                    recovery.delta_filament_mm = recovered;
                    motion.extrusion_spans.push_back(recovery);
                    span.start_fraction = recovery.end_fraction;
                    span.delta_filament_mm = delta - recovered;
                    motion.intent = MotionIntent::mixed;
                }
            }
        }
        Require(std::isfinite(retraction_debt_mm_), DiagnosticCode::invalid_number, "Retraction accounting overflow.");
        motion.extrusion_spans.push_back(span);
        Operation op;
        op.kind = OperationKind::motion;
        op.motion.has_value = true;
        op.motion.value = std::move(motion);
        Add(std::move(op));
        pose_.value = end;
        logical_e_mm_.value += delta;
        filament_mm_.value += delta;
    }

    void ResetOrigin(const Line& line) {
        Allowed(line, "XYZE");
        Require(!line.parameters.empty(), DiagnosticCode::malformed_input, "G92 needs an explicit axis.");
        const char axes[3] = {'X', 'Y', 'Z'};
        for(std::size_t i = 0; i < 3; ++i) if(Has(line, axes[i])){
            Require(pose_.has_value, DiagnosticCode::undefined_modal_state, "G92 cannot establish an unknown physical starting pose.");
            xyz_origin_mm_[i] = pose_.value.position_m[i] * 1000.0 - Length(Get(line, axes[i]));
            Require(std::isfinite(xyz_origin_mm_[i]), DiagnosticCode::invalid_number, "Origin arithmetic overflow.");
        }
        if(Has(line, 'E')){ logical_e_mm_.has_value = true; logical_e_mm_.value = Length(Get(line, 'E')); }
        Info(DiagnosticCode::state_change, "G92 changed logical origins; physical path and commanded filament position are unchanged.");
    }

    void Temperature(const Line& line, bool bed, bool wait) {
        Allowed(line, bed ? (wait ? "SR" : "S") : (wait ? "SRT" : "ST"));
        ZeroChannel(line, 'T');
        Require(Has(line, 'S') != Has(line, 'R'), DiagnosticCode::malformed_input, "Temperature needs exactly one supported S/R target.");
        TemperatureOperation temperature;
        temperature.heater = bed ? HeaterKind::bed : HeaterKind::hotend;
        temperature.target_deg_c = Get(line, Has(line, 'S') ? 'S' : 'R');
        Require(temperature.target_deg_c >= 0, DiagnosticCode::invalid_number, "Negative heater target is unsupported.");
        if(wait){
            temperature.wait_mode.has_value = true;
            temperature.wait_mode.value = Has(line, 'S') ? TemperatureWaitMode::heat_only : TemperatureWaitMode::heat_or_cool;
        }
        Operation op;
        op.kind = wait ? OperationKind::wait_temperature : OperationKind::set_temperature;
        op.temperature.has_value = true;
        op.temperature.value = temperature;
        Add(std::move(op));
    }

    void Execute(const Line& line) {
        if(!line.has_command){
            Require(motion_mode_, DiagnosticCode::undefined_modal_state, "A parameter-only line requires G0/G1 mode.");
            Move(line);
            return;
        }
        if(line.family == 'T'){
            Require(line.command == 0 && line.parameters.empty(), DiagnosticCode::unsupported_command, "Only T0 is supported.");
            Info(DiagnosticCode::state_change, "T0 retained as a single-tool selection; no device action.");
        }else if(line.family == 'G'){
            switch(line.command){
            case 0: case 1: motion_mode_ = true; Move(line); break;
            case 20: case 21:
                Allowed(line, ""); units_ = line.command == 20 ? SourceLengthUnit::inches : SourceLengthUnit::millimetres;
                Info(DiagnosticCode::state_change, "Input units changed; normalized state retained."); break;
            case 90: case 91:
                Allowed(line, ""); xyz_mode_ = e_mode_ = line.command == 90 ? PositionMode::absolute : PositionMode::relative;
                Info(DiagnosticCode::state_change, "XYZ/E positioning modes changed."); break;
            case 92: ResetOrigin(line); break;
            case 4: {
                Allowed(line, "PS");
                Require(Has(line, 'P') != Has(line, 'S'), DiagnosticCode::malformed_input, "G4 needs exactly one P/S duration.");
                const double duration = Has(line, 'P') ? Get(line, 'P') : Get(line, 'S');
                const double seconds = Has(line, 'P') ? duration / 1000.0 : duration;
                Require(seconds >= 0 && std::isfinite(seconds), DiagnosticCode::invalid_number, "Invalid dwell duration.");
                Require(duration == 0 || seconds > 0, DiagnosticCode::invalid_number, "Dwell duration underflow.");
                Operation op;
                if(seconds == 0){ op.kind = OperationKind::barrier; op.barrier.has_value = true; }
                else{ op.kind = OperationKind::dwell; op.dwell.has_value = true; op.dwell.value.duration_s = seconds; }
                Add(std::move(op)); break;
            }
            default: throw ImportError(DiagnosticCode::unsupported_command, "Unsupported G command.");
            }
        }else if(line.family == 'M'){
            switch(line.command){
            case 82: case 83:
                Allowed(line, ""); e_mode_ = line.command == 82 ? PositionMode::absolute : PositionMode::relative;
                Info(DiagnosticCode::state_change, "E positioning override changed."); break;
            case 104: Temperature(line, false, false); break;
            case 109: Temperature(line, false, true); break;
            case 140: Temperature(line, true, false); break;
            case 190: Temperature(line, true, true); break;
            case 106: case 107: {
                Allowed(line, line.command == 106 ? "SP" : "P"); ZeroChannel(line, 'P');
                Require(line.command != 106 || Has(line, 'S'), DiagnosticCode::malformed_input, "M106 needs an S speed.");
                const double speed = line.command == 106 ? Get(line, 'S') : 0;
                Require(speed >= 0 && speed <= 255, DiagnosticCode::invalid_number, "Fan S must be in [0,255].");
                Operation op; op.kind = OperationKind::set_fan; op.fan.has_value = true; op.fan.value.fraction = speed / 255.0;
                Add(std::move(op)); break;
            }
            case 400: {
                Allowed(line, ""); Operation op; op.kind = OperationKind::barrier; op.barrier.has_value = true; Add(std::move(op)); break;
            }
            case 117: Info(DiagnosticCode::metadata, "M117 message retained as inert source metadata."); break;
            case 73:
                Allowed(line, "PR");
                Require(!line.parameters.empty() && (!Has(line, 'P') || (Get(line, 'P') >= 0 && Get(line, 'P') <= 100))
                    && (!Has(line, 'R') || Get(line, 'R') >= 0), DiagnosticCode::invalid_number, "Invalid M73 progress metadata.");
                Info(DiagnosticCode::metadata, "M73 progress retained as inert metadata."); break;
            default: throw ImportError(DiagnosticCode::unsupported_command, "Unsupported M command.");
            }
        }
    }

public:
    explicit Importer(const GCodeParseRequest& request)
        : request_(request), units_(request.options.initial_state.units), xyz_mode_(request.options.initial_state.xyz_mode),
          e_mode_(request.options.initial_state.extrusion_mode), pose_(request.options.initial_state.nozzle_pose_in_bed),
          logical_e_mm_(request.options.initial_state.logical_e_mm), filament_mm_(request.options.initial_state.filament_position_mm) {}

    Result<PrintJob> Run() {
        Result<PrintJob> result;
        try{
            location_.synthetic = true;
            location_.command = "import_request";
            Require(request_.options.maximum_input_bytes > 0 && request_.options.maximum_input_bytes <= 64u * 1024u * 1024u
                && request_.options.maximum_line_bytes > 0 && request_.options.maximum_line_bytes <= 4096
                && request_.options.maximum_operations >= 1 && request_.options.maximum_operations <= 1000000
                && request_.options.maximum_diagnostics >= 1 && request_.options.maximum_diagnostics <= 10000,
                DiagnosticCode::invalid_configuration, "Resource limits exceed the supported import envelope.");
            Require(request_.text.size() <= request_.options.maximum_input_bytes, DiagnosticCode::limit_exceeded, "Input byte limit exceeded.");
            Require(request_.options.dialect_id == "marlin_restricted_v1" && request_.source.dialect_id == request_.options.dialect_id,
                DiagnosticCode::invalid_configuration, "Unsupported or mismatched dialect.");
            Require(units_ == SourceLengthUnit::unspecified || units_ == SourceLengthUnit::millimetres || units_ == SourceLengthUnit::inches,
                DiagnosticCode::invalid_configuration, "Invalid initial unit enum.");
            const auto valid_mode = [](PositionMode mode){ return mode == PositionMode::unspecified || mode == PositionMode::absolute || mode == PositionMode::relative; };
            Require(valid_mode(xyz_mode_) && valid_mode(e_mode_), DiagnosticCode::invalid_configuration, "Invalid positioning enum.");
            ValidateEncoding(request_.text);
            job_.source = request_.source;
            const auto hash = ComputeSourceSha256(request_.text);
            Require(hash.status == ResultStatus::success, DiagnosticCode::identity_mismatch, "Source hashing failed.");
            Require(job_.source.sha256.empty() || job_.source.sha256 == hash.value, DiagnosticCode::identity_mismatch, "Provided source digest does not match the input bytes.");
            job_.source.sha256 = hash.value;
            const auto& initial = request_.options.initial_state;
            if(pose_.has_value){
                for(double value : pose_.value.position_m) Require(std::isfinite(value), DiagnosticCode::invalid_configuration, "Initial pose must be finite.");
                const auto& q = pose_.value.orientation_xyzw;
                const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
                Require(std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-10
                    && std::abs(q.x) <= 1 && std::abs(q.y) <= 1 && std::abs(q.z) <= 1 && std::abs(q.w) <= 1,
                    DiagnosticCode::invalid_configuration, "Initial orientation must be a unit quaternion.");
            }
            if(logical_e_mm_.has_value) Require(std::isfinite(logical_e_mm_.value), DiagnosticCode::invalid_configuration, "Initial logical E must be finite.");
            if(filament_mm_.has_value) Require(std::isfinite(filament_mm_.value), DiagnosticCode::invalid_configuration, "Initial commanded filament position must be finite.");
            if(initial.feedrate_source_units_per_min.has_value){
                feed_mm_per_min_.has_value = true;
                feed_mm_per_min_.value = Length(initial.feedrate_source_units_per_min.value);
                Require(feed_mm_per_min_.value > 0, DiagnosticCode::invalid_configuration, "Initial feedrate must be positive.");
            }
            std::ostringstream seed;
            seed.imbue(std::locale::classic()); seed << std::setprecision(17);
            seed << "initial_state units=" << static_cast<int>(units_) << " xyz=" << static_cast<int>(xyz_mode_) << " e=" << static_cast<int>(e_mode_);
            seed << " pose=" << pose_.has_value;
            if(pose_.has_value){ for(double value : pose_.value.position_m) seed << ',' << value; const auto& q = pose_.value.orientation_xyzw; seed << ',' << q.x << ',' << q.y << ',' << q.z << ',' << q.w; }
            seed << " logical_e=" << logical_e_mm_.has_value << ':' << (logical_e_mm_.has_value ? logical_e_mm_.value : 0);
            seed << " filament=" << filament_mm_.has_value << ':' << (filament_mm_.has_value ? filament_mm_.value : 0);
            seed << " feed=" << feed_mm_per_min_.has_value << ':' << (feed_mm_per_min_.has_value ? feed_mm_per_min_.value : 0);
            location_.synthetic = true; location_.command = "initial_state";
            Info(DiagnosticCode::metadata, seed.str());
            const auto id = ComputeSourceSha256("cdpr-import-v1\n" + hash.value + "\n" + seed.str());
            Require(id.status == ResultStatus::success, DiagnosticCode::identity_mismatch, "Job identity hashing failed.");
            job_.job_id = "import-" + id.value;
            std::size_t start = request_.text.compare(0, 3, "\xEF\xBB\xBF") == 0 ? 3 : 0;
            while(start < request_.text.size()){
                const auto newline = request_.text.find('\n', start);
                auto end = newline == std::string::npos ? request_.text.size() : newline;
                if(newline != std::string::npos && end > start && request_.text[end - 1] == '\r') --end;
                ++physical_line_;
                location_ = SourceLocation{};
                location_.first_line = location_.last_line = physical_line_;
                Require(end - start <= request_.options.maximum_line_bytes, DiagnosticCode::limit_exceeded, "Line byte limit exceeded.");
                const auto raw = request_.text.substr(start, end - start);
                location_.command = raw;
                Require(raw.find('\r') == std::string::npos, DiagnosticCode::malformed_input, "Bare CR line endings are unsupported.");
                const auto code = VerifyEnvelope(raw);
                location_.layer_index = layer_;
                if(!code.empty()) Execute(ParseLine(code));
                if(newline == std::string::npos) break;
                start = newline + 1;
            }
            Require(!job_.operations.empty(), DiagnosticCode::malformed_input, "Input contains no printing operations.");
            Require(job_.operations.size() < request_.options.maximum_operations, DiagnosticCode::limit_exceeded, "No room for end_job.");
            Operation end;
            end.id = job_.operations.size(); end.kind = OperationKind::end_job;
            end.source.synthetic = true; end.source.command = "end_of_file";
            job_.operations.push_back(end);
            for(int i = static_cast<int>(ValidationCheckKind::syntax); i <= static_cast<int>(ValidationCheckKind::stop_interlocks); ++i){
                ValidationCheck check; check.kind = static_cast<ValidationCheckKind>(i);
                if(check.kind == ValidationCheckKind::syntax || check.kind == ValidationCheckKind::modal_state) check.status = ValidationStatus::passed;
                job_.validation.checks.push_back(check);
            }
            const auto validated = ValidateImportedJob(job_);
            if(validated.status != ResultStatus::success){ result.status = ResultStatus::rejected; result.diagnostics = validated.diagnostics; return result; }
            result.status = ResultStatus::success;
            result.value = std::move(job_);
        }catch(const ImportError& error){
            result.status = ResultStatus::rejected;
            Diagnostic diagnostic; diagnostic.code = error.code; diagnostic.message = error.what();
            diagnostic.source.has_value = true; diagnostic.source.value = location_;
            result.diagnostics.push_back(std::move(diagnostic));
        }catch(const std::exception& error){
            result.status = ResultStatus::fault;
            Diagnostic diagnostic; diagnostic.code = DiagnosticCode::execution_fault; diagnostic.message = error.what();
            result.diagnostics.push_back(std::move(diagnostic));
        }
        return result;
    }
};

} // namespace

Result<PrintJob> ParseGCode(const GCodeParseRequest& request) {
    return Importer(request).Run();
}

} // namespace cdpr_slicer
