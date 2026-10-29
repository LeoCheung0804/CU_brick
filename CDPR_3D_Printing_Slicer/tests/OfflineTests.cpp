// Test source added in S2-A. Compilation/execution require S2-C approval.
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include "cdpr_slicer/Planner.h"
#include "json.hpp"
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef CDPR_SLICER_FIXTURE_DIRECTORY
#error CMake must supply the fixture directory.
#endif

namespace {
using namespace cdpr_slicer;
using Json = nlohmann::json;
void Expect(bool condition, const char* message) {
    if(!condition) throw std::runtime_error(message);
}
void Close(double actual, double expected, const char* message) {
    Expect(std::isfinite(actual) && std::abs(actual - expected) <= 1e-12 * std::max(1.0,std::abs(expected)), message);
}
std::string Fixture(const char* name) {
    const std::string path = std::string(CDPR_SLICER_FIXTURE_DIRECTORY) + "/" + name;
    std::ifstream input(path, std::ios::binary);
    Expect(input.good(), "Cannot read fixture.");
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}
GCodeParseRequest Request(const std::string& text) {
    GCodeParseRequest request;
    request.text = text; request.source.file_name = "synthetic.gcode";
    auto& state = request.options.initial_state;
    // These are synthetic, explicitly supplied test coordinates, not calibration.
    state.nozzle_pose_in_bed.has_value = true;
    state.logical_e_mm.has_value = true; state.filament_position_mm.has_value = true;
    return request;
}
PrintJob Accepted(const std::string& text) {
    auto result = ParseGCode(Request(text));
    if(result.status != ResultStatus::success) {
        if(!result.diagnostics.empty()) std::cerr << result.diagnostics.front().message << '\n';
        throw std::runtime_error("Expected a complete accepted import.");
    }
    Expect(!result.value.validation.hardware_ready, "Importer asserted hardware readiness.");
    return result.value;
}
void Rejected(const std::string& text) {
    const auto result = ParseGCode(Request(text));
    Expect(result.status == ResultStatus::rejected, "Expected a rejected import.");
    Expect(result.value.operations.empty() && result.value.job_id.empty(), "Rejected import exposed a partial job.");
    Expect(!result.diagnostics.empty(), "Rejected import has no diagnostic.");
}
std::vector<const MotionOperation*> Motions(const PrintJob& job) {
    std::vector<const MotionOperation*> result;
    for(const auto& op : job.operations) if(op.motion.has_value) result.push_back(&op.motion.value);
    return result;
}
Json ImportConfig() {
    return {{"dialect_id","marlin_restricted_v1"}, {"initial_state",{
        {"units","unspecified"},{"xyz_mode","unspecified"},{"extrusion_mode","unspecified"},
        {"nozzle_pose_in_bed",nullptr},{"logical_e_mm",nullptr},{"filament_position_mm",nullptr},{"feedrate_source_units_per_min",nullptr}}},
        {"maximum_input_bytes",67108864},{"maximum_line_bytes",4096},{"maximum_operations",1000000},{"maximum_diagnostics",10000}};
}

void ModalFixture() {
    const auto job = Accepted(Fixture("modal.gcode"));
    const auto motion = Motions(job);
    Expect(job.operations.size() == 15 && motion.size() == 6, "Fixture operation count changed.");
    Close(motion[0]->nozzle.end.position_m[0],0.01,"Millimeter position was not converted to meters.");
    Close(motion[0]->nozzle.end.position_m[2],0.0002,"Z conversion failed.");
    Close(motion[0]->path_feedrate_m_per_s.value,0.01,"XYZ feedrate conversion failed.");
    Expect(motion[1]->intent == MotionIntent::retract && !motion[1]->path_feedrate_m_per_s.has_value, "E-only retraction was misclassified.");
    Close(motion[1]->filament_feedrate_mm_per_s.value,2.0,"E-only feedrate has the wrong dimension.");
    Expect(motion[2]->intent == MotionIntent::mixed && motion[2]->extrusion_spans.size() == 2, "Recovery/deposition split was lost.");
    Close(motion[2]->nozzle.start.position_m[0],0.01,"G92 moved the physical nozzle.");
    Close(motion[2]->nozzle.end.position_m[0],0.015,"G92 logical origin was not applied.");
    Close(motion[2]->extrusion_spans[0].delta_filament_mm,0.5,"Recovery debt changed.");
    Close(motion[2]->extrusion_spans[0].end_fraction,2.0/3.0,"Recovery fraction changed.");
    Close(motion[2]->extrusion_spans[1].delta_filament_mm,0.25,"Deposited E was not preserved.");
    Close(motion[3]->nozzle.end.position_m[0],0.02,"Relative XYZ failed.");
    Close(motion[4]->nozzle.end.position_m[0],0.0454,"Inch-relative XYZ failed.");
    Close(motion[4]->delta_filament_mm,0.254,"Inch filament conversion failed.");
    Close(motion[4]->path_feedrate_m_per_s.value,0.0254,"Inch feedrate conversion failed.");
    // G92 X0 established a +10 mm physical offset, retained by subsequent G90.
    // Thus the absolute X50 command is physically X60, not X50.
    Close(motion[5]->nozzle.end.position_m[0],0.06,"G92 physical offset was not retained.");
    Close(motion[5]->filament_position_end_mm,2.5,"G92 E reset broke cumulative physical E.");
    Expect(job.operations[0].source.first_line == 6, "Physical source-line tracking failed.");
    Expect(job.operations[6].temperature.value.wait_mode.has_value == false, "Set-temperature acquired a wait.");
    Expect(job.operations[7].temperature.value.wait_mode.value == TemperatureWaitMode::heat_or_cool, "R wait mode was lost.");
    Expect(job.operations[9].temperature.value.wait_mode.value == TemperatureWaitMode::heat_only, "S wait mode was lost.");
    Close(job.operations[10].fan.value.fraction,128.0/255.0,"Fan conversion failed.");
    Close(job.operations[11].dwell.value.duration_s,0.25,"Dwell conversion failed.");
    Expect(job.operations.back().kind == OperationKind::end_job && job.operations.back().source.synthetic, "Final synthetic end_job missing.");
    Expect(job.validation.checks.size() == 17, "Incomplete validation categories.");
    for(const auto& check : job.validation.checks) {
        const bool syntax = check.kind == ValidationCheckKind::syntax || check.kind == ValidationCheckKind::modal_state;
        Expect(check.status == (syntax ? ValidationStatus::passed : ValidationStatus::not_evaluated), "Importer asserted a later validation result.");
    }
}
void UnitModesAndExtrusion() {
    const auto job = Accepted("G21\nG90\nM83\nG1 X10 E1 F600\nG91\nG1 X5 E1\nG90\nG1 X20 E3\nG20\nG1 X1 E0.2\n");
    const auto m = Motions(job);
    Close(m[1]->delta_filament_mm,1,"G91 did not select relative E.");
    Close(m[2]->delta_filament_mm,1,"G90 did not restore absolute E.");
    Close(m[3]->delta_filament_mm,2.08,"Absolute inch E was misread.");
    Close(m[3]->path_feedrate_m_per_s.value,0.01,"A unit switch rescaled an already normalized F.");
    const auto e_only = Accepted("G21\nG90\nM83\nG1 E1 F60\nG1 E-0.5\nG1 E0.25\nG1 E0.5\n");
    const auto e = Motions(e_only);
    Expect(e[0]->intent == MotionIntent::prime && e[2]->intent == MotionIntent::recover && e[3]->intent == MotionIntent::mixed, "E-only prime/recovery classification failed.");
    Expect(e[3]->extrusion_spans[1].purpose == ExtrusionPurpose::prime, "E-only recovery remainder became deposit.");
    const auto zero = Accepted("G21\nG90\nG1 F600\nG1 X0 E0\nG4 P0\nM400\n");
    Expect(Motions(zero).empty() && zero.operations.size() == 3, "No-op/barrier normalization failed.");
    // Adjacent E/e always begins an axis word; it is never an exponent.
    const auto adjacent = Accepted("g21\ng90\ng1x1e-3f60\n");
    Close(Motions(adjacent)[0]->delta_filament_mm,-3,"Adjacent E was interpreted as an exponent.");
}
void RejectionCases() {
    Rejected(Fixture("rejected.gcode"));
    const std::string prefix = "G21\nG90\nM82\n";
    const std::vector<std::string> bad = {
        "G2 X1 Y1 I1 F60","G28","G29","M84","M200 D1.75","M900 K0.1","T1",
        "G1 X1 X2 F60","G1 X F60","G1 Xnan F60","G1 X1 G1 Y2 F60",
        "G1 X1 F0","G1 X1 F-60","G1 X1 A5 F60","G4 P1 S1","G4 S-1","G4 S1e3",
        "M104","M104 S200 T1","M109 S200 R200","M106","M106 S256","M106 S100 P1","M107 S0",
        "G92","G1 X1 F60 (unterminated","G1 X1 F60 (nested(comment))","G1 X1 F60 )","M73 P101",
        "G1 X1.2.3 F60","G1 X1 F60\nM999"
    };
    for(const auto& line : bad) Rejected(prefix + line + "\n");
    Rejected("G1 X1 E1 F60\n");
    Rejected("; comments only\nG21\nG90\n");
    auto unknown = Request(prefix + "G1 X1 F60\n");
    unknown.options.initial_state.nozzle_pose_in_bed.has_value = false;
    Expect(ParseGCode(unknown).status == ResultStatus::rejected,"Unknown starting pose was fabricated.");
    unknown = Request(prefix + "G92 E0\nG1 X1 F60\n");
    unknown.options.initial_state.filament_position_mm.has_value = false;
    Expect(ParseGCode(unknown).status == ResultStatus::rejected,"G92 fabricated a physical filament datum.");
    unknown = Request(prefix + "G1 X1 F60\n"); unknown.options.initial_state.logical_e_mm.has_value = false;
    Expect(ParseGCode(unknown).status == ResultStatus::rejected,"Travel fabricated logical E.");
    auto bad_orientation = Request(prefix + "G1 X1 F60\n");
    bad_orientation.options.initial_state.nozzle_pose_in_bed.value.orientation_xyzw.w = 0;
    Expect(ParseGCode(bad_orientation).status == ResultStatus::rejected,"Nonunit orientation was accepted.");
}
void SourceEnvelopes() {
    const auto numbered = Accepted("N1 G21*27\nN2 G90*18\nN3 M82*26\nN4 G92 E0*67\nN5 G1 X1 F60*4\n");
    Expect(numbered.operations[0].source.first_line == 5,"N numbers replaced physical source lines.");
    Rejected("N1 G21*26\nN2 G90*18\n");
    Rejected("N1 G21*27\nN2 G90\nG1 X1 F60\n");
    Rejected("N1 G21\nN3 G90\nG1 X1 F60\n");
    Rejected("N1 G21\nG90\n");
    Rejected("G21*27\n");
    const auto bom = Accepted("\xEF\xBB\xBFg21\r\ng90\r\n;LAYER:2\r\ng1x1f60 (comment)\r\n");
    Expect(bom.operations[0].source.layer_index.has_value && bom.operations[0].source.layer_index.value == 2,"Layer annotation lost.");
    std::string nul = "G21\nG90\nG1 X1 F60"; nul.push_back('\0'); nul += "\n"; Rejected(nul);
    Rejected(std::string("G21\nG90\n;") + "\xC0\xAF" + "\nG1 X1 F60\n");
    Rejected("G21\rG90\nG1 X1 F60\n");
    Rejected("G21\nG90\nG1 X1 F60\r");
    Rejected(std::string("G21\nG90\n;") + '\x7F' + "\nG1 X1 F60\n");
    auto limited = Request("G21\nG90\nG1 X1 F60\n");
    limited.options.maximum_input_bytes = 4; Expect(ParseGCode(limited).status == ResultStatus::rejected,"Input bound ignored.");
    limited = Request("G21\nG90\nG1 X1 F60\n"); limited.options.maximum_line_bytes = 4;
    Expect(ParseGCode(limited).status == ResultStatus::rejected,"Line bound ignored.");
    limited = Request("G21\nG90\nG1 X1 F60\n"); limited.options.maximum_operations = 1;
    Expect(ParseGCode(limited).status == ResultStatus::rejected,"end_job was excluded from operation bound.");
    limited.options.maximum_operations = 2;
    Expect(ParseGCode(limited).status == ResultStatus::success,"Two-operation envelope rejected a move plus end_job.");
    limited.options.maximum_diagnostics = 1;
    Expect(ParseGCode(limited).status == ResultStatus::rejected,"Diagnostic bound ignored.");
}
void DigestsAndIdentity() {
    const std::vector<std::pair<std::string,std::string>> vectors = {
        {"","e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
        {"abc","ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"},
        {"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq","248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"},
        {std::string(1000000,'a'),"cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"}
    };
    for(const auto& vector : vectors) {
        const auto digest = ComputeSourceSha256(vector.first);
        Expect(digest.status == ResultStatus::success && digest.value == vector.second,"SHA-256 known-answer vector failed.");
    }
    auto request = Request("G21\nG90\nG1 X1 F60\n");
    const auto first = ParseGCode(request); const auto repeat = ParseGCode(request);
    Expect(first.status == ResultStatus::success && repeat.value.job_id == first.value.job_id,"Import identity is not deterministic.");
    request.options.initial_state.nozzle_pose_in_bed.value.position_m[1] = 0.001;
    const auto different = ParseGCode(request);
    Expect(different.status == ResultStatus::success && different.value.job_id != first.value.job_id,"Initial state omitted from identity.");
    request.source.sha256 = std::string(64,'0');
    Expect(ParseGCode(request).status == ResultStatus::rejected,"Incorrect supplied source hash was accepted.");
}
void JobRoundTripAndTampering() {
    const auto original = Accepted(Fixture("modal.gcode"));
    const auto serialized = SerializePrintJob(original);
    Expect(serialized.status == ResultStatus::success,"Valid imported job failed serialization.");
    const auto decoded = DeserializePrintJob(serialized.value);
    Expect(decoded.status == ResultStatus::success,"Round trip failed.");
    const auto second = SerializePrintJob(decoded.value);
    Expect(second.status == ResultStatus::success && second.value == serialized.value,"Round trip changed the job.");
    const auto base = Json::parse(serialized.value);
    const std::vector<std::function<void(Json&)>> changes = {
        [](Json& j){ j["unexpected"] = true; },
        [](Json& j){ j.erase("source"); },
        [](Json& j){ j["schema_version"] = 2; },
        [](Json& j){ j["stage"] = "timed"; },
        [](Json& j){ j["validation"]["hardware_ready"] = true; },
        [](Json& j){ j["validation"]["checks"][2]["status"] = "passed"; },
        [](Json& j){ j["validation"]["checks"][1]["kind"] = "syntax"; },
        [](Json& j){ j["operations"][0]["id"] = -1; },
        [](Json& j){ j["operations"][0]["id"] = 0.0; },
        [](Json& j){ j["operations"][0]["id"] = 9007199254740992ULL; },
        [](Json& j){ j["operations"][0]["motion"]["tool_id"] = true; },
        [](Json& j){ j["operations"][0]["motion"]["nozzle"]["end"]["orientation_xyzw"]["w"] = 0; },
        [](Json& j){ j["operations"][0]["motion"]["nozzle"]["frame_id"] = "cdpr_world"; },
        [](Json& j){ j["operations"][0]["motion"]["delta_filament_mm"] = 2; },
        [](Json& j){ j["operations"][2]["motion"]["extrusion_spans"][0]["end_fraction"] = 0.5; },
        [](Json& j){ j["operations"][1]["motion"]["filament_position_start_mm"] = 0; },
        [](Json& j){ j["operations"][1]["motion"]["nozzle"]["start"]["position_m"][0] = 10; },
        [](Json& j){ j["operations"][0]["fan"] = {{"channel",0},{"fraction",1}}; },
        [](Json& j){ j["operations"].erase(j["operations"].end()-1); },
        [](Json& j){ j["source"]["sha256"] = "fabricated"; },
        [](Json& j){ j["frames"]["status"] = "verified"; },
        [](Json& j){ j["operations"][7]["temperature"]["wait_mode"] = nullptr; }
    };
    for(const auto& change : changes) {
        auto tampered = base; change(tampered);
        const auto result = DeserializePrintJob(tampered.dump());
        Expect(result.status == ResultStatus::rejected && result.value.operations.empty(),"Tampered job was accepted or returned partial data.");
    }
    auto duplicate = serialized.value; duplicate.insert(duplicate.find('{') + 1, "\"schema_version\":1,");
    Expect(DeserializePrintJob(duplicate).status == ResultStatus::rejected,"Duplicate JSON key accepted.");
    std::string nested(40,'['); nested += "0"; nested += std::string(40,']');
    Expect(DeserializePrintJob(nested).status == ResultStatus::rejected,"JSON depth bound ignored.");
    auto memory = original; memory.operations[0].motion.value.delta_filament_mm = std::numeric_limits<double>::infinity();
    Expect(SerializePrintJob(memory).status == ResultStatus::rejected,"Nonfinite data became JSON null.");
    memory = original; memory.operations[0].motion.value.intent = static_cast<MotionIntent>(999);
    Expect(SerializePrintJob(memory).status == ResultStatus::rejected,"Invalid in-memory enum serialized.");
    Expect(SerializePrintJob(PrintJob{}).status == ResultStatus::rejected,"Incomplete default job serialized successfully.");
}
void ConfigurationAndSyntheticFixture() {
    const auto config = ImportConfig();
    const auto read = DeserializeImportOptions(config.dump());
    Expect(read.status == ResultStatus::success && !read.value.initial_state.nozzle_pose_in_bed.has_value,"Unknown initial state acquired a default pose.");
    auto bad = config; bad["unknown"] = 1;
    Expect(DeserializeImportOptions(bad.dump()).status == ResultStatus::rejected,"Unknown import option ignored.");
    bad = config; bad["maximum_operations"] = -1;
    Expect(DeserializeImportOptions(bad.dump()).status == ResultStatus::rejected,"Negative operation bound wrapped.");
    bad = config; bad["maximum_input_bytes"] = 67108865;
    Expect(DeserializeImportOptions(bad.dump()).status == ResultStatus::rejected,"Oversized import bound accepted.");
    bad = config; bad["initial_state"]["feedrate_source_units_per_min"] = 10;
    Expect(DeserializeImportOptions(bad.dump()).status == ResultStatus::rejected,"Feedrate with unknown units accepted.");
    auto duplicate = config.dump(); duplicate.insert(1,"\"dialect_id\":\"marlin_restricted_v1\",");
    Expect(DeserializeImportOptions(duplicate).status == ResultStatus::rejected,"Duplicate import option accepted.");
    const auto fixture = Json::parse(Fixture("robot-model.json"));
    Expect(fixture.at("synthetic") == true && fixture.at("hardware_access") == false,"Geometry fixture is not explicitly synthetic/offline.");
    Expect(fixture.at("kinematics").at("frame_attachments_m").size() == 8,"Synthetic geometry fixture shape changed.");
    // This is a fixture integrity check, not a robot-model loader or kinematics test.
}
void CoordinateConventions() {
    FrameRegistration frames; frames.status=RegistrationStatus::verified;
    frames.world_from_bed.has_value=true; frames.end_effector_from_nozzle.has_value=true;
    frames.world_from_bed.value.translation_m={{1,2,3}};
    frames.world_from_bed.value.rotation_xyzw.z=std::sqrt(0.5); frames.world_from_bed.value.rotation_xyzw.w=std::sqrt(0.5);
    frames.end_effector_from_nozzle.value.translation_m={{0.1,0,0}};
    CartesianPose nozzle; nozzle.position_m={{0.2,0.3,0.4}};
    const auto transformed=TransformNozzleToEndEffector(nozzle,frames);
    Expect(transformed.status==ResultStatus::success,"Explicit rigid transform failed.");
    Close(transformed.value.position_m[0],0.7,"World/bed composition X failed.");
    Close(transformed.value.position_m[1],2.1,"Nozzle offset was not rotated/inverted.");
    Close(transformed.value.position_m[2],3.4,"World/bed composition Z failed.");
    const Eigen::Quaterniond q=Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitX())
        *Eigen::AngleAxisd(-0.3,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(0.4,Eigen::Vector3d::UnitZ());
    CartesianPose oriented; oriented.orientation_xyzw.x=q.x(); oriented.orientation_xyzw.y=q.y();
    oriented.orientation_xyzw.z=q.z(); oriented.orientation_xyzw.w=q.w();
    const auto legacy=ToLegacyPose(oriented);
    Expect(legacy.status==ResultStatus::success,"XYZ Euler conversion failed.");
    Close(legacy.value[3],0.2,"Roll convention changed."); Close(legacy.value[4],-0.3,"Pitch convention changed."); Close(legacy.value[5],0.4,"Yaw convention changed.");
    oriented.orientation_xyzw=Quaternion{}; oriented.orientation_xyzw.y=std::sqrt(0.5); oriented.orientation_xyzw.w=std::sqrt(0.5);
    Expect(ToLegacyPose(oriented).status==ResultStatus::rejected,"Euler gimbal lock was accepted.");
    frames.world_from_bed.value.rotation_xyzw.w=0;
    Expect(TransformNozzleToEndEffector(nozzle,frames).status==ResultStatus::rejected,"Nonunit calibration was silently normalized.");
}
void PreviewEscaping() {
    auto request=Request("G21\nG90\nG1 X1 F60\nM117 </script><script>alert(1)</script>\n");
    request.source.file_name="</script><script>untrusted</script>__CDPR_ROBOT_DATA__";
    const auto parsed=ParseGCode(request); Expect(parsed.status==ResultStatus::success,"Synthetic preview input failed.");
    const auto preview=CreatePreviewHtml(parsed.value);
    Expect(preview.status==ResultStatus::success,"Imported preview generation failed.");
    Expect(preview.value.find("</script><script>alert(1)")==std::string::npos
        &&preview.value.find("</script><script>untrusted")==std::string::npos,"Untrusted source escaped the inert JSON block.");
    Expect(preview.value.find("\\u003c/script\\u003e")!=std::string::npos,"HTML delimiters were not escaped.");
    Expect(preview.value.find("hardware_ready=false")!=std::string::npos,"Preview omitted its planning status.");
    const auto robot_start=preview.value.find("<script id=\"robotdata\" type=\"application/json\">");
    Expect(robot_start!=std::string::npos&&preview.value.substr(robot_start).find(">null</script>")!=std::string::npos,
        "User data redirected preview template substitution or invented a robot.");
}
#ifdef _WIN32
#ifndef CDPR_SLICER_TEST_DIRECTORY
#error CMake must supply a dedicated build/test-data directory.
#endif
std::string NewTestFile(const Json& content) {
    const std::string directory=CDPR_SLICER_TEST_DIRECTORY;
    const auto bytes=content.dump(2);
    static unsigned sequence=0;
    for(unsigned attempt=0;attempt<64;++attempt) {
        const auto path=directory+"/case-"+std::to_string(GetCurrentProcessId())+"-"+std::to_string(sequence++)+".json";
        const int count=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.data(),static_cast<int>(path.size()),nullptr,0);
        Expect(count>0,"Test path conversion failed."); std::wstring wide(static_cast<std::size_t>(count),L'\0');
        Expect(MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.data(),static_cast<int>(path.size()),&wide[0],count)==count,"Test path conversion failed.");
        std::replace(wide.begin(),wide.end(),L'/',L'\\');
        Expect(wide.size()>3&&wide[1]==L':'&&wide[2]==L'\\',"Test output requires a local build path.");
        for(std::size_t end=wide.find(L'\\',3);end!=std::wstring::npos;end=wide.find(L'\\',end+1)) {
            const auto attributes=GetFileAttributesW(wide.substr(0,end).c_str());
            Expect(attributes!=INVALID_FILE_ATTRIBUTES&&(attributes&FILE_ATTRIBUTE_DIRECTORY)&&!(attributes&FILE_ATTRIBUTE_REPARSE_POINT),"Test build ancestors must be real directories.");
        }
        const auto literal=L"\\\\?\\"+wide;
        const HANDLE file=CreateFileW(literal.c_str(),GENERIC_WRITE,0,nullptr,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(file==INVALID_HANDLE_VALUE) {
            const auto error=GetLastError();
            if(error==ERROR_FILE_EXISTS||error==ERROR_ALREADY_EXISTS) continue;
            throw std::runtime_error("Cannot create a new file in build/test-data.");
        }
        DWORD written=0; const bool ok=WriteFile(file,bytes.data(),static_cast<DWORD>(bytes.size()),&written,nullptr)!=0;
        const bool closed=CloseHandle(file)!=0;
        Expect(ok&&written==bytes.size()&&closed,"Generated test fixture write failed."); return path;
    }
    throw std::runtime_error("No unused generated test filename.");
}
Json SyntheticLimits() {
    const auto empty=SerializeMotionLimits(MotionLimits{}); Expect(empty.status==ResultStatus::success,"Empty limit encoding failed.");
    auto limits=Json::parse(empty.value);
    limits["workspace"]={{"minimum_m",{-0.5,-0.5,-0.5}},{"maximum_m",{0.5,0.5,0.5}}};
    limits["maximum_path_speed_m_per_s"]=0.04; limits["maximum_path_acceleration_m_per_s2"]=0.1;
    limits["minimum_cable_length_m"]=std::array<double,8>{{0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1}};
    limits["maximum_cable_length_m"]=std::array<double,8>{{4,4,4,4,4,4,4,4}};
    limits["maximum_cable_speed_m_per_s"]=std::array<double,8>{{0.03,0.03,0.03,0.03,0.03,0.03,0.03,0.03}};
    limits["maximum_cable_acceleration_m_per_s2"]=std::array<double,8>{{0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08}};
    limits["minimum_motor_count"]=std::array<std::int32_t,8>{{0,0,0,0,0,0,0,0}};
    limits["maximum_motor_count"]=std::array<std::int32_t,8>{{500000,500000,500000,500000,500000,500000,500000,500000}};
    limits["maximum_filament_rate_mm_per_s"]=2; limits["maximum_volumetric_flow_mm3_per_s"]=8;
    return limits;
}
ModelLoadRequest SyntheticRequest(const Json* supplied_limits=nullptr) {
    ModelLoadRequest request; request.robot_config_path=std::string(CDPR_SLICER_FIXTURE_DIRECTORY)+"/robot-model.json";
    request.rail_offsets_m.has_value=true; request.starting_rail_offsets_m.has_value=true;
    FrameRegistration frames; frames.status=RegistrationStatus::verified;
    frames.world_from_bed.has_value=true; frames.world_from_bed.value.translation_m={{0.1,0.2,0}};
    frames.end_effector_from_nozzle.has_value=true; frames.end_effector_from_nozzle.value.translation_m={{0,0,0.05}};
    const auto serialized=SerializeFrameRegistration(frames); Expect(serialized.status==ResultStatus::success,"Synthetic registration encoding failed.");
    const Json calibration={{"schema_version",1},{"frames",Json::parse(serialized.value)},
        {"provenance",{{"calibration_id","synthetic-test-only"},{"method","mathematical fixture; not machine calibration"}}}};
    const Json manifest={{"schema_version",1},{"geometry_contract","cu_brick_fixed_rails_v1"},{"limits",supplied_limits?*supplied_limits:SyntheticLimits()}};
    request.calibration_manifest_path.has_value=true; request.calibration_manifest_path.value=NewTestFile(calibration);
    request.motion_limits_manifest_path.has_value=true; request.motion_limits_manifest_path.value=NewTestFile(manifest);
    return request;
}
RobotModelSnapshot SyntheticModel(const Json* limits=nullptr) {
    const auto loaded=LoadRobotModel(SyntheticRequest(limits));
    if(loaded.status!=ResultStatus::success) {
        if(!loaded.diagnostics.empty()) std::cerr<<loaded.diagnostics.front().message<<'\n';
        throw std::runtime_error("Synthetic model loading failed.");
    }
    return loaded.value;
}
PlanningOptions SyntheticOptions() {
    PlanningOptions options;
    options.sample_period_s.has_value=true; options.sample_period_s.value=0.02;
    options.path_tolerance_m.has_value=true; options.path_tolerance_m.value=0.003;
    options.filament_diameter_mm.has_value=true; options.filament_diameter_mm.value=1.75;
    options.temperature_wait_policy.hotend.tolerance_deg_c.has_value=true; options.temperature_wait_policy.hotend.tolerance_deg_c.value=2;
    options.temperature_wait_policy.hotend.timeout_s.has_value=true; options.temperature_wait_policy.hotend.timeout_s.value=60;
    options.temperature_wait_policy.bed=options.temperature_wait_policy.hotend; return options;
}
Json PreviewBlock(const std::string& html,const std::string& name) {
    const auto marker="<script id=\""+name+"\" type=\"application/json\">";
    const auto offset=html.find(marker); Expect(offset!=std::string::npos,"Missing inert preview data.");
    const auto start=offset+marker.size(),end=html.find("</script>",start);
    Expect(end!=std::string::npos,"Unterminated inert preview data.");
    return Json::parse(html.substr(start,end-start));
}
void ModelBound3DPreview() {
    const auto model=SyntheticModel(); const auto imported=Accepted(Fixture("modal.gcode"));
    const auto planned=PlanTrajectory(imported,model,SyntheticOptions());
    Expect(planned.status==ResultStatus::success,"Preview planning fixture failed.");
    const auto before=SerializePrintJob(planned.value); const auto preview=CreatePreviewHtml(planned.value,model);
    Expect(preview.status==ResultStatus::success,"Model-bound 3D preview failed.");
    Expect(SerializePrintJob(planned.value).value==before.value,"Preview modified the artifact.");
    Expect(PreviewBlock(preview.value,"jobdata")==Json::parse(before.value),"3D preview changed embedded plan/checks.");
    const auto data=PreviewBlock(preview.value,"robotdata");
    Expect(data.at("frame_anchors_m").size()==8&&data.at("ee_attachments_m").size()==8,"Preview did not bind eight cable endpoints.");
    Expect(data.at("model_sha256")==model.identity.robot_config_sha256,"Preview omitted model provenance.");
    Expect(data.at("cable_samples").size()==6,"Preview cable samples lost operation mapping.");
    const auto& points=data.at("cable_samples")[0].at("samples");
    const auto& motion=planned.value.operations[0].motion.value.end_effector.value;
    for(const bool last:{false,true}) {
        const auto legacy=ToLegacyPose(last?motion.end:motion.start);
        const auto lengths=ComputeCheckedCableLengths(model,legacy.value);
        const auto& actual=last?points.back():points.front();
        Close(actual.at("fraction").get<double>(),last?1:0,"Preview sample omitted exact endpoint.");
        for(std::size_t cable=0;cable<8;++cable)
            Close(actual.at("lengths_m")[cable].get<double>(),lengths.value[cable],"Preview cable length differs from the shared solver.");
    }
    auto changed=model; changed.rail_offsets_m[0]=0.1;
    Expect(CreatePreviewHtml(planned.value,changed).status==ResultStatus::rejected,"Unsealed preview model was accepted.");
    auto other_limits=SyntheticLimits(); other_limits["maximum_filament_rate_mm_per_s"]=1;
    const auto other=SyntheticModel(&other_limits);
    Expect(CreatePreviewHtml(planned.value,other).status==ResultStatus::rejected,"Different valid model was attached to an existing job.");
    Expect(CreatePreviewHtml(imported,model).status==ResultStatus::rejected,"Imported path acquired unregistered robot geometry.");
    const auto path_only=CreatePreviewHtml(planned.value);
    Expect(path_only.status==ResultStatus::success&&PreviewBlock(path_only.value,"robotdata").is_null(),"Path-only overload fabricated robot geometry.");
}
void ModelProvenanceAndLegacyScales() {
    auto request=SyntheticRequest(); const auto model=LoadRobotModel(request);
    Expect(model.status==ResultStatus::success&&ValidateModelSnapshot(model.value).status==ResultStatus::success,"Model snapshot seal failed.");
    auto negative_zero=request;
    negative_zero.rail_offsets_m.value[0]=-0.0;negative_zero.starting_rail_offsets_m.value[0]=-0.0;
    const auto normalized=LoadRobotModel(negative_zero);
    Expect(normalized.status==ResultStatus::success&&normalized.value.identity.rail_state_sha256==model.value.identity.rail_state_sha256,
        "Rail identity did not canonicalize negative zero.");
    auto changed=model.value; changed.cable_scale_counts_per_m[0]+=1;
    Expect(ValidateModelSnapshot(changed).status==ResultStatus::rejected,"Changed scale retained a valid snapshot seal.");
    changed=model.value; changed.rail_offsets_m[0]=0.1;
    Expect(ValidateModelSnapshot(changed).status==ResultStatus::rejected,"Changed rail state retained its identity.");
    request.calibration_manifest_path.has_value=false; request.motion_limits_manifest_path.has_value=false;
    const auto provisional=LoadRobotModel(request);
    Expect(provisional.status==ResultStatus::success&&!provisional.value.has_calibration
        &&BindPlanningOptions(SyntheticOptions(),provisional.value).status==ResultStatus::rejected,"Missing provenance produced a bound planning model.");
    request.expected_robot_config_sha256.has_value=true; request.expected_robot_config_sha256.value=std::string(64,'0');
    Expect(LoadRobotModel(request).status==ResultStatus::rejected,"Stale robot file hash was accepted.");
    request.expected_robot_config_sha256.has_value=false; request.rail_offsets_m.has_value=false;
    Expect(LoadRobotModel(request).status==ResultStatus::rejected,"Missing rail state acquired zero defaults.");
    const auto fixture=Json::parse(Fixture("robot-model.json")); const auto& k=fixture.at("kinematics");
    Json legacy={{"cableMotorNum",8},{"railMotorNum",4},{"pulleyRadius",0.01},{"isOnline",true},
        {"frameAttachments",k.at("frame_attachments_m")},{"endEffectorAttachments",k.at("end_effector_attachments_m")},
        {"pulleyZdir",{-1,-1,-1,-1,1,1,1,1}},{"cableMotorScale",100000.75},{"endEffVelLmt",0.45},
        {"cableMotorScaleIndividual",{123456.789,123456.789,123456.789,123456.789,123456.789,123456.789,123456.789,123456.789}},
        {"endEffectorPosLimit",{{"x",-0.5},{"y",0.5},{"z",-0.5},{"roll",0.5},{"pitch",-0.5},{"yaw",0.5}}},
        {"useIndividualCableScale",false}};
    request=ModelLoadRequest{}; request.rail_offsets_m.has_value=true; request.starting_rail_offsets_m.has_value=true;
    request.robot_config_path=NewTestFile(legacy); const auto common=LoadRobotModel(request);
    Expect(common.status==ResultStatus::success&&common.value.cable_scale_counts_per_m[0]==100000,"Disabled individual scales overrode the common scale.");
    Expect(common.value.limits.maximum_path_speed_m_per_s.value==static_cast<double>(static_cast<float>(0.45)),"Path-speed float conversion differs from Robot.");
    legacy["useIndividualCableScale"]=true; request.robot_config_path=NewTestFile(legacy);
    const auto individual=LoadRobotModel(request);
    Expect(individual.status==ResultStatus::success&&individual.value.cable_scale_counts_per_m[0]==static_cast<double>(static_cast<float>(123456.789)),"Individual scale float conversion differs from Robot.");
    legacy.erase("cableMotorScaleIndividual"); request.robot_config_path=NewTestFile(legacy);
    Expect(LoadRobotModel(request).status==ResultStatus::rejected,"Enabled but absent individual scales were fabricated.");
}
void PlanningAndArtifactRoundTrip() {
    const auto model=SyntheticModel(); const auto options=SyntheticOptions();
    const auto imported=Accepted(Fixture("modal.gcode")); const auto planned=PlanTrajectory(imported,model,options);
    if(planned.status!=ResultStatus::success) {if(!planned.diagnostics.empty())std::cerr<<planned.diagnostics.front().message<<'\n';throw std::runtime_error("Synthetic timing failed.");}
    Expect(planned.value.stage==PlanningStage::timed&&!planned.value.validation.hardware_ready,"Timed plan asserted readiness or incorrect stage.");
    Expect(planned.value.operations.size()==imported.operations.size(),"Planner dropped or duplicated source operations.");
    Expect(planned.value.job_id!=imported.job_id,"Plan identity reused the import ID.");
    std::uint64_t epoch=0; double cursor=0;
    for(std::size_t i=0;i<planned.value.operations.size();++i) {
        const auto& op=planned.value.operations[i]; const auto& timing=op.timing.value;
        Expect(op.id==imported.operations[i].id&&op.source.command==imported.operations[i].source.command,"Planning changed source identity/order.");
        Expect(op.timing.has_value&&timing.epoch_id==epoch,"Epoch order failed."); Close(timing.start_s,cursor,"Epoch time continuity failed.");
        if(op.motion.has_value) {
            const auto& m=op.motion.value; Close(m.delta_filament_mm,imported.operations[i].motion.value.delta_filament_mm,"Retiming changed commanded E.");
            const auto& a=timing.profile.front(); const auto& b=timing.profile.back();
            Expect(a.progress==0&&b.progress==1&&a.filament_rate_mm_per_s==0&&b.filament_rate_mm_per_s==0,"Quintic endpoints/rest state were lost.");
            double length=0;for(std::size_t j=0;j<3;++j)length=std::hypot(length,m.nozzle.end.position_m[j]-m.nozzle.start.position_m[j]);
            Expect(1.875*length/timing.duration_s<=0.04*(1+1e-10),"Analytic path speed limit exceeded.");
            Expect((10*std::sqrt(3.0)/3)*length/(timing.duration_s*timing.duration_s)<=0.1*(1+1e-10),"Analytic path acceleration limit exceeded.");
            Expect(1.875*std::abs(m.delta_filament_mm)/timing.duration_s<=2*(1+1e-10),"Analytic filament rate limit exceeded.");
        }
        cursor+=timing.duration_s;if(op.kind==OperationKind::wait_temperature){++epoch;cursor=0;}
    }
    for(const auto& check:planned.value.validation.checks) if(check.kind==ValidationCheckKind::tension||check.kind==ValidationCheckKind::collision
        ||check.kind==ValidationCheckKind::singularity||check.kind==ValidationCheckKind::cable_acceleration||check.kind==ValidationCheckKind::feedback)
        Expect(check.status==ValidationStatus::not_evaluated,"Sampled planning overclaimed physical/continuous validation.");
    const auto encoded=SerializePrintJob(planned.value);Expect(encoded.status==ResultStatus::success,"Timed plan serialization failed.");
    const auto decoded=DeserializePrintJob(encoded.value);Expect(decoded.status==ResultStatus::success,"Timed plan decoding failed.");
    Expect(SerializePrintJob(decoded.value).value==encoded.value,"Timed job round trip changed data.");
    auto bad=Json::parse(encoded.value);bad["operations"][0]["timing"]["profile"][1]["progress"]=0.9;
    Expect(DeserializePrintJob(bad.dump()).status==ResultStatus::rejected,"A knot inconsistent with quintic interpolation was accepted.");
    Expect(CreatePreviewHtml(planned.value).status==ResultStatus::success,"Timed preview generation failed.");
}
void GeometricAndPlanningRejections() {
    const auto model=SyntheticModel(); auto options=SyntheticOptions();const auto job=Accepted("G21\nG90\nG1 X10 E1 F600\n");
    auto bad_options=options;bad_options.sample_period_s.has_value=false;
    const auto missing=PlanTrajectory(job,model,bad_options);
    Expect(missing.status==ResultStatus::indeterminate&&missing.value.operations.empty(),"Missing cadence fabricated a timed plan.");
    bad_options=options;bad_options.limits.maximum_path_speed_m_per_s.has_value=true;bad_options.limits.maximum_path_speed_m_per_s.value=0.02;
    Expect(PlanTrajectory(job,model,bad_options).status==ResultStatus::rejected,"Unmanifested option override was accepted.");
    const auto outside=Accepted("G21\nG90\nG1 X1000 E1 F600\n");
    Expect(PlanTrajectory(outside,model,options).status==ResultStatus::rejected,"Outside-workspace path was accepted.");
    auto limits=SyntheticLimits();limits["maximum_cable_acceleration_m_per_s2"]=nullptr;const auto incomplete=SyntheticModel(&limits);
    const auto not_timed=PlanTrajectory(job,incomplete,options);
    Expect(not_timed.status==ResultStatus::indeterminate&&not_timed.value.operations.empty(),"Unknown cable acceleration was treated as safe.");
    const auto geometric=CreateGeometricPlan(job,incomplete,options);
    Expect(geometric.status==ResultStatus::success&&geometric.value.stage==PlanningStage::geometrically_checked,"Unknown dynamic limits prevented a labelled geometric artifact.");
    bad_options=options;bad_options.maximum_samples=5;bad_options.maximum_samples_per_motion=5;
    Expect(PlanTrajectory(job,model,bad_options).status==ResultStatus::rejected,"Sample exhaustion returned a partial plan.");
    auto degenerate=std::array<double,6>{{0,0,0,0,0,0}};
    const Eigen::Vector3d point=model.kinematics.frameAttachments[0]-model.kinematics.endEffectorAttachments[0];
    for(std::size_t i=0;i<3;++i)degenerate[i]=point[static_cast<int>(i)];
    Expect(ComputeCheckedCableLengths(model,degenerate).status==ResultStatus::rejected,"Degenerate pulley plane reached unchecked math.");
    auto slow_limits=SyntheticLimits();
    slow_limits["maximum_cable_speed_m_per_s"]=std::array<double,8>{{0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001}};
    const auto slow_model=SyntheticModel(&slow_limits);
    const auto short_job=Accepted("G21\nG90\nG1 X1 F600\n");
    const auto normal=PlanTrajectory(short_job,model,options), slower=PlanTrajectory(short_job,slow_model,options);
    Expect(normal.status==ResultStatus::success&&slower.status==ResultStatus::success,"Synthetic cable-limited retiming failed.");
    Expect(slower.value.operations[0].timing.value.duration_s>normal.value.operations[0].timing.value.duration_s*1.1,"Tighter cable speed did not slow the motion.");
    Expect(slower.value.operations[0].motion.value.nozzle.end.position_m==normal.value.operations[0].motion.value.nozzle.end.position_m,"Cable retiming moved an endpoint.");
    const auto geometric_normal=CreateGeometricPlan(short_job,model,options);
    Expect(geometric_normal.status==ResultStatus::success,"Intermediate geometry creation failed.");
    const auto resumed=PlanTrajectory(geometric_normal.value,model,options);
    Expect(resumed.status==ResultStatus::success&&resumed.value.job_id==normal.value.job_id,"Planning a saved geometric stage changed the resulting content identity.");
}
#endif
} // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {
        {"modal fixture",ModalFixture}, {"unit modes and extrusion",UnitModesAndExtrusion},
        {"rejection cases",RejectionCases}, {"source envelopes",SourceEnvelopes},
        {"digests and identity",DigestsAndIdentity}, {"job round trip and tampering",JobRoundTripAndTampering},
        {"configuration and synthetic fixture",ConfigurationAndSyntheticFixture},
        {"coordinate conventions",CoordinateConventions},{"preview escaping",PreviewEscaping},
#ifdef _WIN32
        {"model provenance and legacy scales",ModelProvenanceAndLegacyScales},
        {"model-bound 3D preview",ModelBound3DPreview},
        {"planning and artifact round trip",PlanningAndArtifactRoundTrip},
        {"geometry/planning rejections",GeometricAndPlanningRejections}
#endif
    };
    unsigned failures = 0;
    for(const auto& test : tests) {
        try { test.run(); std::cout << "PASS " << test.name << '\n'; }
        catch(const std::exception& error) { ++failures; std::cerr << "FAIL " << test.name << ": " << error.what() << '\n'; }
    }
    return failures == 0 ? 0 : 1;
}
