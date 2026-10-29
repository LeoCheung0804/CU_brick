#include "cdpr_slicer/Planner.h"
#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {

using Json = nlohmann::json;
constexpr std::size_t kMaximumJsonBytes = 64u * 1024u * 1024u;
constexpr std::uint64_t kMaximumIndex = 9007199254740991ULL;

struct ContractError : std::runtime_error {
    DiagnosticCode code;
    ContractError(DiagnosticCode c, const std::string& message) : std::runtime_error(message), code(c) {}
};
void Check(bool condition, const std::string& message, DiagnosticCode code = DiagnosticCode::malformed_input) {
    if(!condition) throw ContractError(code, message);
}
template<typename T, typename F> Result<T> Attempt(F action) {
    Result<T> result;
    try {
        result.value = action();
        result.status = ResultStatus::success;
    } catch(const ContractError& error) {
        result.status = ResultStatus::rejected;
        Diagnostic d; d.code = error.code; d.message = error.what();
        result.diagnostics.push_back(std::move(d));
    } catch(const Json::exception&) {
        result.status = ResultStatus::rejected;
        Diagnostic d; d.code = DiagnosticCode::malformed_input; d.message = "Invalid JSON syntax, encoding, type or numeric range.";
        result.diagnostics.push_back(std::move(d));
    } catch(const std::exception&) {
        result.status = ResultStatus::fault;
        Diagnostic d; d.code = DiagnosticCode::execution_fault; d.message = "The offline operation could not complete.";
        result.diagnostics.push_back(std::move(d));
    }
    return result;
}

template<typename E> const std::vector<std::pair<E, const char*>>& EnumValues();
#define CDPR_ENUM_TABLE(E, ...) \
    template<> const std::vector<std::pair<E, const char*>>& EnumValues<E>() { \
        static const std::vector<std::pair<E, const char*>> values{__VA_ARGS__}; return values; }
#define CDPR_ENUM(E, V) {E::V, #V}
CDPR_ENUM_TABLE(Severity, CDPR_ENUM(Severity, info), CDPR_ENUM(Severity, warning), CDPR_ENUM(Severity, error), CDPR_ENUM(Severity, critical))
CDPR_ENUM_TABLE(DiagnosticCode,
    CDPR_ENUM(DiagnosticCode, metadata), CDPR_ENUM(DiagnosticCode, state_change), CDPR_ENUM(DiagnosticCode, no_op),
    CDPR_ENUM(DiagnosticCode, not_implemented), CDPR_ENUM(DiagnosticCode, malformed_input), CDPR_ENUM(DiagnosticCode, unsupported_command),
    CDPR_ENUM(DiagnosticCode, undefined_modal_state), CDPR_ENUM(DiagnosticCode, invalid_number), CDPR_ENUM(DiagnosticCode, invalid_configuration),
    CDPR_ENUM(DiagnosticCode, invalid_transform), CDPR_ENUM(DiagnosticCode, invalid_geometry), CDPR_ENUM(DiagnosticCode, outside_workspace),
    CDPR_ENUM(DiagnosticCode, limit_exceeded), CDPR_ENUM(DiagnosticCode, infeasible_tension), CDPR_ENUM(DiagnosticCode, missing_model_data),
    CDPR_ENUM(DiagnosticCode, identity_mismatch), CDPR_ENUM(DiagnosticCode, invalid_timing), CDPR_ENUM(DiagnosticCode, stale_feedback),
    CDPR_ENUM(DiagnosticCode, capability_unavailable), CDPR_ENUM(DiagnosticCode, queue_fault), CDPR_ENUM(DiagnosticCode, communication_fault),
    CDPR_ENUM(DiagnosticCode, thermal_fault), CDPR_ENUM(DiagnosticCode, execution_fault))
CDPR_ENUM_TABLE(MotionIntent, CDPR_ENUM(MotionIntent, travel), CDPR_ENUM(MotionIntent, deposit), CDPR_ENUM(MotionIntent, retract),
    CDPR_ENUM(MotionIntent, recover), CDPR_ENUM(MotionIntent, prime), CDPR_ENUM(MotionIntent, mixed))
CDPR_ENUM_TABLE(ExtrusionPurpose, CDPR_ENUM(ExtrusionPurpose, none), CDPR_ENUM(ExtrusionPurpose, deposit),
    CDPR_ENUM(ExtrusionPurpose, retract), CDPR_ENUM(ExtrusionPurpose, recover), CDPR_ENUM(ExtrusionPurpose, prime))
CDPR_ENUM_TABLE(HeaterKind, CDPR_ENUM(HeaterKind, hotend), CDPR_ENUM(HeaterKind, bed))
CDPR_ENUM_TABLE(TemperatureWaitMode, CDPR_ENUM(TemperatureWaitMode, heat_only), CDPR_ENUM(TemperatureWaitMode, heat_or_cool))
CDPR_ENUM_TABLE(BarrierReason, CDPR_ENUM(BarrierReason, explicit_sync), CDPR_ENUM(BarrierReason, pause_boundary), CDPR_ENUM(BarrierReason, job_end))
CDPR_ENUM_TABLE(OperationKind, CDPR_ENUM(OperationKind, motion), CDPR_ENUM(OperationKind, dwell),
    CDPR_ENUM(OperationKind, set_temperature), CDPR_ENUM(OperationKind, wait_temperature), CDPR_ENUM(OperationKind, set_fan),
    CDPR_ENUM(OperationKind, barrier), CDPR_ENUM(OperationKind, end_job))
CDPR_ENUM_TABLE(ValidationStatus, CDPR_ENUM(ValidationStatus, not_evaluated), CDPR_ENUM(ValidationStatus, passed), CDPR_ENUM(ValidationStatus, failed))
CDPR_ENUM_TABLE(ValidationCheckKind, CDPR_ENUM(ValidationCheckKind, syntax), CDPR_ENUM(ValidationCheckKind, modal_state),
    CDPR_ENUM(ValidationCheckKind, coordinate_transform), CDPR_ENUM(ValidationCheckKind, geometry),
    CDPR_ENUM(ValidationCheckKind, workspace), CDPR_ENUM(ValidationCheckKind, orientation), CDPR_ENUM(ValidationCheckKind, motor_count),
    CDPR_ENUM(ValidationCheckKind, cable_speed), CDPR_ENUM(ValidationCheckKind, cable_acceleration), CDPR_ENUM(ValidationCheckKind, collision),
    CDPR_ENUM(ValidationCheckKind, singularity), CDPR_ENUM(ValidationCheckKind, tension), CDPR_ENUM(ValidationCheckKind, execution_timing),
    CDPR_ENUM(ValidationCheckKind, feedback), CDPR_ENUM(ValidationCheckKind, thermal), CDPR_ENUM(ValidationCheckKind, extrusion_sync),
    CDPR_ENUM(ValidationCheckKind, stop_interlocks))
CDPR_ENUM_TABLE(SourceLengthUnit, CDPR_ENUM(SourceLengthUnit, unspecified), CDPR_ENUM(SourceLengthUnit, millimetres), CDPR_ENUM(SourceLengthUnit, inches))
CDPR_ENUM_TABLE(PositionMode, CDPR_ENUM(PositionMode, unspecified), CDPR_ENUM(PositionMode, absolute), CDPR_ENUM(PositionMode, relative))
CDPR_ENUM_TABLE(PlanningStage, CDPR_ENUM(PlanningStage, imported), CDPR_ENUM(PlanningStage, transformed),
    CDPR_ENUM(PlanningStage, geometrically_checked), CDPR_ENUM(PlanningStage, timed), CDPR_ENUM(PlanningStage, simulated))
#undef CDPR_ENUM
#undef CDPR_ENUM_TABLE
template<typename E> std::string Name(E value) {
    for(const auto& item : EnumValues<E>()) if(item.first == value) return item.second;
    throw ContractError(DiagnosticCode::malformed_input, "Unknown enum value.");
}
template<typename E> E ReadEnum(const Json& value) {
    Check(value.is_string(), "Enum must be a string.");
    const auto token = value.get<std::string>();
    for(const auto& item : EnumValues<E>()) if(token == item.second) return item.first;
    throw ContractError(DiagnosticCode::malformed_input, "Unknown enum token.");
}

void Text(const std::string& value, std::size_t minimum, std::size_t maximum) {
    Check(value.size() >= minimum && value.size() <= maximum, "String byte limit violated.");
    // Check UTF-8 even for an in-memory job supplied without going through JSON.
    for(std::size_t i = 0; i < value.size();) {
        const auto a = static_cast<unsigned char>(value[i]);
        Check(a != 0, "NUL is forbidden in strings.");
        if(a < 128) { ++i; continue; }
        const std::size_t n = a >= 0xC2 && a <= 0xDF ? 2 : a >= 0xE0 && a <= 0xEF ? 3 : a >= 0xF0 && a <= 0xF4 ? 4 : 0;
        Check(n && n <= value.size() - i, "Invalid UTF-8 string.");
        for(std::size_t j = 1; j < n; ++j) {
            const auto b = static_cast<unsigned char>(value[i + j]);
            Check(b >= 0x80 && b <= 0xBF, "Invalid UTF-8 continuation.");
        }
        const auto b = static_cast<unsigned char>(value[i + 1]);
        Check(!(a == 0xE0 && b < 0xA0) && !(a == 0xED && b >= 0xA0)
            && !(a == 0xF0 && b < 0x90) && !(a == 0xF4 && b > 0x8F), "Invalid UTF-8 scalar.");
        i += n;
    }
}
std::string String(const Json& value, std::size_t minimum = 1, std::size_t maximum = 128) {
    Check(value.is_string(), "Expected a string.");
    auto text = value.get<std::string>(); Text(text, minimum, maximum); return text;
}
void Keys(const Json& value, std::initializer_list<const char*> keys) {
    Check(value.is_object() && value.size() == keys.size(), "Missing or extra object fields.");
    for(const auto key : keys) Check(value.contains(key), "Missing required field.");
}
void Array(const Json& value, std::size_t minimum, std::size_t maximum) {
    Check(value.is_array() && value.size() >= minimum && value.size() <= maximum, "Array size is outside the supported envelope.");
}
double Number(const Json& value) {
    Check(value.is_number(), "Expected a finite number.");
    const auto number = value.get<double>();
    Check(std::isfinite(number), "Nonfinite number."); return number;
}
void Finite(double number) { Check(std::isfinite(number), "Nonfinite number."); }
std::uint64_t Index(const Json& value, std::uint64_t maximum = kMaximumIndex) {
    Check(value.is_number_integer(), "Expected a nonnegative integer literal.");
    if(value.is_number_unsigned()) {
        const auto n = value.get<std::uint64_t>(); Check(n <= maximum, "Integer exceeds range."); return n;
    }
    const auto n = value.get<std::int64_t>();
    Check(n >= 0 && static_cast<std::uint64_t>(n) <= maximum, "Integer exceeds range.");
    return static_cast<std::uint64_t>(n);
}
bool Boolean(const Json& value) { Check(value.is_boolean(), "Expected a boolean."); return value.get<bool>(); }
bool Near(double a, double b) {
    return std::isfinite(a) && std::isfinite(b)
        && (a == b || std::abs(a - b) <= 32 * std::numeric_limits<double>::epsilon() * std::max(std::abs(a), std::abs(b)));
}
void Hash(const std::string& value) {
    Check(value.size() == 64 && std::all_of(value.begin(), value.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); }), "Expected a lowercase SHA-256 digest.");
}
void FrameId(const std::string& value) {
    Text(value, 1, 128);
    const auto letter = [](char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); };
    Check(letter(value.front()) && std::all_of(value.begin(), value.end(), [&](char c) {
        return letter(c) || (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-'; }), "Invalid frame identifier.");
}
void Pose(const CartesianPose& pose) {
    for(double value : pose.position_m) Finite(value);
    const auto& q = pose.orientation_xyzw;
    Finite(q.x); Finite(q.y); Finite(q.z); Finite(q.w);
    Check(std::abs(q.x) <= 1 && std::abs(q.y) <= 1 && std::abs(q.z) <= 1 && std::abs(q.w) <= 1
        && std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) <= 1e-10, "Orientation must be a unit quaternion.");
}
bool SameOrientation(const Quaternion& a, const Quaternion& b) {
    // Preserve the source representation, including quaternion sign.
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
bool SamePose(const CartesianPose& a, const CartesianPose& b) {
    return a.position_m == b.position_m && SameOrientation(a.orientation_xyzw, b.orientation_xyzw);
}
void Location(const SourceLocation& source) {
    Text(source.command, 0, 4096);
    Check(source.first_line <= kMaximumIndex && source.last_line <= kMaximumIndex, "Source line index exceeds range.");
    Check(source.synthetic ? source.first_line == 0 && source.last_line == 0
        : source.first_line >= 1 && source.last_line >= source.first_line, "Invalid source range.");
    if(source.layer_index.has_value) Check(source.layer_index.value <= kMaximumIndex, "Layer index exceeds range.");
}
void Diagnostics(const std::vector<Diagnostic>& diagnostics, std::size_t operation_count, std::size_t& total) {
    Check(diagnostics.size() <= 10000 && total <= 10000 - diagnostics.size(), "Diagnostic limit exceeded.");
    total += diagnostics.size();
    for(const auto& diagnostic : diagnostics) {
        Name(diagnostic.severity); Name(diagnostic.code); Text(diagnostic.message, 1, 4096);
        Check(diagnostic.severity == Severity::info || diagnostic.severity == Severity::warning, "An accepted import cannot contain error diagnostics.");
        if(diagnostic.operation_id.has_value) Check(diagnostic.operation_id.value < operation_count, "Diagnostic refers to an absent operation.");
        if(diagnostic.source.has_value) Location(diagnostic.source.value);
    }
}

double PathLength(const MotionOperation& motion) {
    double length = 0;
    for(std::size_t i = 0; i < 3; ++i) length = std::hypot(length,motion.nozzle.end.position_m[i]-motion.nozzle.start.position_m[i]);
    Finite(length); return length;
}
bool ClosePose(const CartesianPose& a, const CartesianPose& b) {
    for(std::size_t i=0;i<3;++i)
        if(std::abs(a.position_m[i]-b.position_m[i]) > 1e-10*(1+std::max(std::abs(a.position_m[i]),std::abs(b.position_m[i])))) return false;
    const auto& p=a.orientation_xyzw; const auto& q=b.orientation_xyzw;
    const auto close=[](double x,double y){return std::abs(x-y)<=1e-10;};
    return (close(p.x,q.x)&&close(p.y,q.y)&&close(p.z,q.z)&&close(p.w,q.w))
        || (close(p.x,-q.x)&&close(p.y,-q.y)&&close(p.z,-q.z)&&close(p.w,-q.w));
}
void Rigid(const RigidTransform& value) {
    CartesianPose p; p.position_m=value.translation_m; p.orientation_xyzw=value.rotation_xyzw; Pose(p);
}
void CheckTiming(const PrintJob& job) {
    std::uint64_t epoch=0; double cursor=0; std::size_t knots=0;
    for(const auto& operation:job.operations) {
        Check(operation.timing.has_value,"Every timed-stage operation needs timing.");
        const auto& timing=operation.timing.value;
        Finite(timing.start_s); Finite(timing.duration_s);
        Check(timing.epoch_id==epoch && Near(timing.start_s,cursor) && timing.duration_s>=0
            && timing.duration_s<=86400,"Invalid epoch, event order or operation duration.");
        if(operation.motion.has_value) {
            Check(timing.interpolation=="quintic_progress_v1" && timing.duration_s>0
                && timing.profile.size()>=2 && timing.profile.size()<=50000,"Invalid quintic motion profile.");
            Check(knots<=200000-timing.profile.size(),"Timing knot limit exceeded."); knots+=timing.profile.size();
            const auto& m=operation.motion.value; const double length=PathLength(m), duration=timing.duration_s;
            const bool moved=length>0;
            Check(std::isfinite(1.875/duration),"Motion duration is too small.");
            const double peak=(moved?length:std::abs(m.delta_filament_mm))*1.875/duration;
            const double requested=moved?m.path_feedrate_m_per_s.value:m.filament_feedrate_mm_per_s.value;
            Check(std::isfinite(peak) && peak<=requested*(1+1e-10),"Quintic peak exceeds requested feedrate.");
            double previous=-1,previous_progress=0;
            for(const auto& knot:timing.profile) {
                Finite(knot.offset_s); Finite(knot.progress); Finite(knot.filament_offset_mm); Finite(knot.filament_rate_mm_per_s);
                Check(knot.offset_s>previous && knot.offset_s>=0 && knot.offset_s<=duration,"Profile offsets must increase within the duration.");
                const double u=knot.offset_s/duration;
                const double s=u==1?1:u*u*u*(10+u*(-15+6*u));
                const double rate=30*u*u*(1-u)*(1-u)/duration;
                const auto close=[](double a,double b){return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-10*(1+std::max(std::abs(a),std::abs(b)));};
                Check(knot.progress>=previous_progress && knot.progress>=0 && knot.progress<=1 && close(knot.progress,s)
                    && close(knot.filament_offset_mm,m.delta_filament_mm*s)
                    && close(knot.filament_rate_mm_per_s,m.delta_filament_mm*rate),"Knot contradicts quintic interpolation or extrusion.");
                Check(knot.path_speed_m_per_s.has_value==moved,"Invalid knot speed dimension.");
                if(moved) { Finite(knot.path_speed_m_per_s.value); Check(knot.path_speed_m_per_s.value>=0
                    && close(knot.path_speed_m_per_s.value,length*rate),"Incorrect quintic path speed."); }
                previous=knot.offset_s;
                previous_progress=knot.progress;
            }
            const auto& first=timing.profile.front(); const auto& last=timing.profile.back();
            Check(first.offset_s==0 && first.progress==0 && first.filament_offset_mm==0
                && first.filament_rate_mm_per_s==0 && last.offset_s==duration && last.progress==1
                && last.filament_offset_mm==m.delta_filament_mm && last.filament_rate_mm_per_s==0,
                "Motion must retain both endpoints and begin/end at rest.");
            if(moved) Check(first.path_speed_m_per_s.value==0&&last.path_speed_m_per_s.value==0,"Cartesian endpoints must be at rest.");
        } else {
            Check(timing.interpolation=="event_v1" && timing.profile.empty(),"Events cannot contain motion knots.");
            Check(timing.duration_s==(operation.dwell.has_value?operation.dwell.value.duration_s:0),"Event duration mismatch.");
        }
        cursor+=timing.duration_s; Finite(cursor); Check(cursor<=604800,"Epoch duration exceeds the planning envelope.");
        if(operation.kind==OperationKind::wait_temperature) { ++epoch; cursor=0; }
    }
}

void CheckJob(const PrintJob& job) {
    Check(job.schema_version == kPrintJobSchemaVersion, "Unsupported print-job schema version.");
    Name(job.stage);
    Check(job.stage != PlanningStage::simulated,"Simulation is not implemented.",DiagnosticCode::not_implemented);
    const bool advanced=job.stage!=PlanningStage::imported;
    const bool checked=job.stage==PlanningStage::geometrically_checked || job.stage==PlanningStage::timed;
    const bool timed=job.stage==PlanningStage::timed;
    Text(job.job_id, 1, 128); Text(job.source.file_name, 1, 1024); Hash(job.source.sha256);
    Check(job.source.dialect_id == "marlin_restricted_v1", "Unsupported source dialect.");
    if(job.source.producer.has_value) { Text(job.source.producer.value.name, 1, 128); Text(job.source.producer.value.version, 1, 128); }
    if(job.source.profile_sha256.has_value) Hash(job.source.profile_sha256.value);
    Check(job.units.position == "m" && job.units.orientation == "quaternion_xyzw" && job.units.filament == "mm"
        && job.units.time == "s" && job.units.temperature == "degC", "Noncanonical units.");
    const auto& frames = job.frames;
    FrameId(frames.world_frame_id); FrameId(frames.bed_frame_id); FrameId(frames.end_effector_frame_id); FrameId(frames.nozzle_frame_id);
    const std::set<std::string> ids{frames.world_frame_id, frames.bed_frame_id, frames.end_effector_frame_id, frames.nozzle_frame_id};
    Check(ids.size() == 4, "Frame identifiers must be distinct.");
    if(!advanced) {
        Check(!job.model_identity.has_value && !frames.world_from_bed.has_value && !frames.end_effector_from_nozzle.has_value
            && frames.status==RegistrationStatus::unverified,"Imported jobs cannot claim model registration.");
    } else {
        Check(job.model_identity.has_value && frames.world_from_bed.has_value && frames.end_effector_from_nozzle.has_value
            && frames.status==RegistrationStatus::verified,"Planning stages require complete identity and registered transforms.");
        const auto& identity=job.model_identity.value;
        Hash(identity.robot_config_sha256); Hash(identity.calibration_sha256); Hash(identity.rail_state_sha256); Hash(identity.motion_limits_sha256);
        Rigid(frames.world_from_bed.value); Rigid(frames.end_effector_from_nozzle.value);
    }
    Check(job.operations.size() >= 2 && job.operations.size() <= 1000000, "An imported job needs operations followed by end_job.");
    Check(!job.validation.hardware_ready, "Imported artifacts cannot be hardware ready.");
    const auto& kinds = EnumValues<ValidationCheckKind>();
    Check(job.validation.checks.size() == kinds.size(), "Every validation category must be represented once.");
    std::set<ValidationCheckKind> seen;
    std::size_t diagnostic_count = 0;
    Diagnostics(job.validation.diagnostics, job.operations.size(), diagnostic_count);
    for(const auto& check : job.validation.checks) {
        Name(check.kind); Name(check.status);
        Check(seen.insert(check.kind).second, "Duplicate validation category.");
        const bool completed=check.kind==ValidationCheckKind::syntax || check.kind==ValidationCheckKind::modal_state
            || (advanced&&(check.kind==ValidationCheckKind::coordinate_transform || check.kind==ValidationCheckKind::orientation))
            || (checked&&(check.kind==ValidationCheckKind::geometry || check.kind==ValidationCheckKind::workspace))
            || (timed&&check.kind==ValidationCheckKind::execution_timing);
        const auto expected = completed
            ? ValidationStatus::passed : ValidationStatus::not_evaluated;
        Check(check.status==expected || (checked&&check.kind==ValidationCheckKind::motor_count&&check.status==ValidationStatus::passed),
            "Validation claims exceed the implemented planning stage.");
        Diagnostics(check.diagnostics, job.operations.size(), diagnostic_count);
    }
    OptionalValue<MotionOperation> previous;
    double debt = 0;
    std::uint64_t last_line = 0;
    for(std::size_t i = 0; i < job.operations.size(); ++i) {
        const auto& op = job.operations[i];
        Check(op.id == i, "Operation IDs must be consecutive from zero."); Name(op.kind); Location(op.source);
        Check(op.timing.has_value==timed,"Only timed-stage operations may have execution timing.");
        const bool end = op.kind == OperationKind::end_job;
        Check(end == (i + 1 == job.operations.size()), "Exactly one end_job must terminate the job.");
        Check(end ? op.source.synthetic : !op.source.synthetic, "Only end_job is synthetic in an imported job.");
        if(!end) {
            Check(op.source.first_line == op.source.last_line && op.source.first_line > last_line, "Imported operations must follow physical source-line order.");
            last_line = op.source.last_line;
        }
        Check(op.motion.has_value == (op.kind == OperationKind::motion)
            && op.dwell.has_value == (op.kind == OperationKind::dwell)
            && op.temperature.has_value == (op.kind == OperationKind::set_temperature || op.kind == OperationKind::wait_temperature)
            && op.fan.has_value == (op.kind == OperationKind::set_fan)
            && op.barrier.has_value == (op.kind == OperationKind::barrier), "Operation payload does not match its kind.");
        if(op.motion.has_value) {
            const auto& m = op.motion.value;
            Name(m.intent); Pose(m.nozzle.start); Pose(m.nozzle.end);
            Check(m.nozzle.frame_id==frames.bed_frame_id && m.end_effector.has_value==advanced && m.tool_id==0,"Invalid motion frame/tool.");
            if(advanced) {
                const auto& ee=m.end_effector.value; Pose(ee.start); Pose(ee.end);
                Check(ee.frame_id==frames.world_frame_id,"Derived EE frame must be the world frame.");
                const auto start=TransformNozzleToEndEffector(m.nozzle.start,frames);
                const auto transformed_end=TransformNozzleToEndEffector(m.nozzle.end,frames);
                Check(start.status==ResultStatus::success && transformed_end.status==ResultStatus::success
                    && ClosePose(ee.start,start.value) && ClosePose(ee.end,transformed_end.value),"Derived EE geometry contradicts the registered transforms.");
                Check(ToLegacyPose(ee.start).status==ResultStatus::success && ToLegacyPose(ee.end).status==ResultStatus::success,"Legacy orientation conversion failed.");
                if(previous.has_value) Check(ClosePose(previous.value.end_effector.value.end,ee.start),"Discontinuous derived EE path.");
            }
            Check(SameOrientation(m.nozzle.start.orientation_xyzw, m.nozzle.end.orientation_xyzw), "Fixed orientation is required.");
            const bool moved = m.nozzle.start.position_m != m.nozzle.end.position_m;
            Finite(m.delta_filament_mm); Finite(m.logical_e_start_mm); Finite(m.logical_e_end_mm);
            Finite(m.filament_position_start_mm); Finite(m.filament_position_end_mm);
            const double delta = m.delta_filament_mm;
            Check(moved || delta != 0, "No-op movements must be omitted with an import diagnostic.");
            Check(Near(m.logical_e_end_mm, m.logical_e_start_mm + delta)
                && Near(m.filament_position_end_mm, m.filament_position_start_mm + delta), "E endpoint accounting mismatch.");
            Check(delta == 0 || (m.logical_e_start_mm != m.logical_e_end_mm
                && m.filament_position_start_mm != m.filament_position_end_mm), "E delta was lost to endpoint precision.");
            Check(m.path_feedrate_m_per_s.has_value == moved && m.filament_feedrate_mm_per_s.has_value != moved, "Motion requires the correct feedrate dimension.");
            const double feed = moved ? m.path_feedrate_m_per_s.value : m.filament_feedrate_mm_per_s.value;
            Finite(feed); Check(feed > 0, "Motion feedrate must be positive.");
            if(previous.has_value) {
                Check(SamePose(previous.value.nozzle.end, m.nozzle.start), "Discontinuous physical nozzle path.");
                Check(previous.value.filament_position_end_mm == m.filament_position_start_mm, "Discontinuous commanded filament position.");
            }
            const double recovery = delta > 0 ? std::min(delta, debt) : 0;
            const bool mixed = recovery > 0 && recovery < delta;
            const auto positive_purpose = moved ? ExtrusionPurpose::deposit : ExtrusionPurpose::prime;
            const auto expected_intent = delta < 0 ? MotionIntent::retract : delta == 0 ? MotionIntent::travel
                : mixed ? MotionIntent::mixed : recovery == delta ? MotionIntent::recover
                : moved ? MotionIntent::deposit : MotionIntent::prime;
            Check(m.intent == expected_intent && m.extrusion_spans.size() == (mixed ? 2u : 1u), "Extrusion intent/spans contradict retraction accounting.");
            double fraction = 0, sum = 0;
            for(std::size_t j = 0; j < m.extrusion_spans.size(); ++j) {
                const auto& span = m.extrusion_spans[j];
                Name(span.purpose); Finite(span.start_fraction); Finite(span.end_fraction); Finite(span.delta_filament_mm);
                Check(span.start_fraction == fraction && span.end_fraction > fraction && span.end_fraction <= 1, "Spans must partition [0,1] in order.");
                const auto purpose = delta < 0 ? ExtrusionPurpose::retract : delta == 0 ? ExtrusionPurpose::none
                    : recovery > 0 && j == 0 ? ExtrusionPurpose::recover : positive_purpose;
                const double expected_delta = mixed ? (j == 0 ? recovery : delta - recovery) : delta;
                const double expected_end = mixed && j == 0 ? recovery / delta : 1;
                Check(span.purpose == purpose && Near(span.delta_filament_mm, expected_delta) && Near(span.end_fraction, expected_end), "Span purpose, delta or boundary mismatch.");
                fraction = span.end_fraction; sum += span.delta_filament_mm;
            }
            Check(fraction == 1 && Near(sum, delta), "Spans do not preserve the full movement.");
            debt = delta < 0 ? debt - delta : debt - recovery; Finite(debt);
            previous.has_value = true; previous.value = m;
        } else if(op.dwell.has_value) {
            Finite(op.dwell.value.duration_s); Check(op.dwell.value.duration_s > 0, "Dwell must have positive duration.");
        } else if(op.temperature.has_value) {
            const auto& t = op.temperature.value;
            Name(t.heater); Finite(t.target_deg_c);
            Check(t.channel == 0 && t.target_deg_c >= 0, "Invalid heater target/channel.");
            Check(t.wait_mode.has_value == (op.kind == OperationKind::wait_temperature), "Invalid temperature wait mode.");
            if(t.wait_mode.has_value) Name(t.wait_mode.value);
            if(timed && op.kind==OperationKind::wait_temperature) {
                Check(t.tolerance_deg_c.has_value && t.timeout_s.has_value,"Timed waits require explicit tolerance/timeout.");
                Finite(t.tolerance_deg_c.value); Finite(t.timeout_s.value);
                Check(t.tolerance_deg_c.value>0 && t.timeout_s.value>0&&t.timeout_s.value<=86400,"Thermal waits require positive limits and a timeout no longer than one day.");
            } else Check(!t.tolerance_deg_c.has_value && !t.timeout_s.has_value,"Thermal wait limits are reserved for timed waits.");
        } else if(op.fan.has_value) {
            Finite(op.fan.value.fraction);
            Check(op.fan.value.channel == 0 && op.fan.value.fraction >= 0 && op.fan.value.fraction <= 1, "Invalid fan event.");
        } else if(op.barrier.has_value) {
            Check(op.barrier.value.reason == BarrierReason::explicit_sync, "Unsupported imported barrier reason.");
        }
    }
    if(timed) CheckTiming(job);
}

// Parser callback rejects duplicate keys before nlohmann's object map can replace
// an earlier value. Object keys arrive one depth below object_start.
Json ParseJson(const std::string& text) {
    Check(text.size() <= kMaximumJsonBytes, "JSON byte limit exceeded.", DiagnosticCode::limit_exceeded);
    std::vector<std::set<std::string>> keys(34);
    auto callback = [&](int depth, Json::parse_event_t event, Json& parsed) {
        Check(depth >= 0 && depth <= 32, "JSON nesting limit exceeded.", DiagnosticCode::limit_exceeded);
        if(event == Json::parse_event_t::object_start) keys[static_cast<std::size_t>(depth) + 1].clear();
        if(event == Json::parse_event_t::key)
            Check(keys[static_cast<std::size_t>(depth)].insert(parsed.get<std::string>()).second, "Duplicate JSON object key.");
        return true;
    };
    return Json::parse(text, callback, true, false);
}
template<typename T, typename F> OptionalValue<T> ReadOptional(const Json& value, F read) {
    OptionalValue<T> result;
    if(!value.is_null()) { result.value = read(value); result.has_value = true; }
    return result;
}
OptionalValue<double> ReadOptionalNumber(const Json& value) { return ReadOptional<double>(value, Number); }
Json OptionalNumber(const OptionalValue<double>& value) { return value.has_value ? Json(value.value) : Json(nullptr); }
Json WritePose(const CartesianPose& p) {
    const auto& q = p.orientation_xyzw;
    return {{"position_m", p.position_m}, {"orientation_xyzw", {{"x",q.x}, {"y",q.y}, {"z",q.z}, {"w",q.w}}}};
}
CartesianPose ReadPose(const Json& value) {
    Keys(value, {"position_m", "orientation_xyzw"}); Array(value.at("position_m"), 3, 3);
    CartesianPose pose;
    for(std::size_t i = 0; i < 3; ++i) pose.position_m[i] = Number(value.at("position_m").at(i));
    const auto& q = value.at("orientation_xyzw"); Keys(q, {"x","y","z","w"});
    pose.orientation_xyzw.x = Number(q.at("x")); pose.orientation_xyzw.y = Number(q.at("y"));
    pose.orientation_xyzw.z = Number(q.at("z")); pose.orientation_xyzw.w = Number(q.at("w"));
    Pose(pose); return pose;
}
Json WriteRigid(const RigidTransform& value) {
    const auto& q=value.rotation_xyzw;
    return {{"translation_m",value.translation_m},{"rotation_xyzw",{{"x",q.x},{"y",q.y},{"z",q.z},{"w",q.w}}}};
}
RigidTransform ReadRigid(const Json& value) {
    Keys(value,{"translation_m","rotation_xyzw"});
    const auto pose=ReadPose(Json{{"position_m",value.at("translation_m")},{"orientation_xyzw",value.at("rotation_xyzw")}});
    RigidTransform result; result.translation_m=pose.position_m; result.rotation_xyzw=pose.orientation_xyzw; return result;
}
Json WriteFrames(const FrameRegistration& f) {
    return {{"world_frame_id",f.world_frame_id},{"bed_frame_id",f.bed_frame_id},{"end_effector_frame_id",f.end_effector_frame_id},
        {"nozzle_frame_id",f.nozzle_frame_id},{"world_from_bed",f.world_from_bed.has_value?WriteRigid(f.world_from_bed.value):Json(nullptr)},
        {"end_effector_from_nozzle",f.end_effector_from_nozzle.has_value?WriteRigid(f.end_effector_from_nozzle.value):Json(nullptr)},
        {"status",f.status==RegistrationStatus::verified?"verified":"unverified"}};
}
FrameRegistration ReadFrames(const Json& f) {
    Keys(f,{"world_frame_id","bed_frame_id","end_effector_frame_id","nozzle_frame_id","world_from_bed","end_effector_from_nozzle","status"});
    FrameRegistration frames;
    frames.world_frame_id=String(f.at("world_frame_id")); frames.bed_frame_id=String(f.at("bed_frame_id"));
    frames.end_effector_frame_id=String(f.at("end_effector_frame_id")); frames.nozzle_frame_id=String(f.at("nozzle_frame_id"));
    FrameId(frames.world_frame_id); FrameId(frames.bed_frame_id); FrameId(frames.end_effector_frame_id); FrameId(frames.nozzle_frame_id);
    Check(std::set<std::string>{frames.world_frame_id,frames.bed_frame_id,frames.end_effector_frame_id,frames.nozzle_frame_id}.size()==4,"Frame IDs must be distinct.");
    frames.world_from_bed=ReadOptional<RigidTransform>(f.at("world_from_bed"),ReadRigid);
    frames.end_effector_from_nozzle=ReadOptional<RigidTransform>(f.at("end_effector_from_nozzle"),ReadRigid);
    const auto status=String(f.at("status")); Check(status=="unverified"||status=="verified","Unknown registration status.");
    frames.status=status=="verified"?RegistrationStatus::verified:RegistrationStatus::unverified;
    Check(frames.status!=RegistrationStatus::verified || (frames.world_from_bed.has_value&&frames.end_effector_from_nozzle.has_value),
        "Verified registration requires both transforms."); return frames;
}
template<typename T> Json OptionalArray(const OptionalValue<T>& value) {return value.has_value?Json(value.value):Json(nullptr);}
Json WriteLimits(const MotionLimits& l) {
    return {{"workspace",l.workspace.has_value?Json{{"minimum_m",l.workspace.value.minimum_m},{"maximum_m",l.workspace.value.maximum_m}}:Json(nullptr)},
        {"maximum_path_speed_m_per_s",OptionalNumber(l.maximum_path_speed_m_per_s)},
        {"maximum_path_acceleration_m_per_s2",OptionalNumber(l.maximum_path_acceleration_m_per_s2)},
        {"minimum_cable_length_m",OptionalArray(l.minimum_cable_length_m)},{"maximum_cable_length_m",OptionalArray(l.maximum_cable_length_m)},
        {"maximum_cable_speed_m_per_s",OptionalArray(l.maximum_cable_speed_m_per_s)},
        {"maximum_cable_acceleration_m_per_s2",OptionalArray(l.maximum_cable_acceleration_m_per_s2)},
        {"minimum_motor_count",OptionalArray(l.minimum_motor_count)},{"maximum_motor_count",OptionalArray(l.maximum_motor_count)},
        {"maximum_filament_rate_mm_per_s",OptionalNumber(l.maximum_filament_rate_mm_per_s)},
        {"maximum_volumetric_flow_mm3_per_s",OptionalNumber(l.maximum_volumetric_flow_mm3_per_s)}};
}
MotionLimits ReadLimits(const Json& v) {
    Keys(v,{"workspace","maximum_path_speed_m_per_s","maximum_path_acceleration_m_per_s2","minimum_cable_length_m","maximum_cable_length_m",
        "maximum_cable_speed_m_per_s","maximum_cable_acceleration_m_per_s2","minimum_motor_count","maximum_motor_count",
        "maximum_filament_rate_mm_per_s","maximum_volumetric_flow_mm3_per_s"});
    MotionLimits l;
    l.workspace=ReadOptional<AxisBounds>(v.at("workspace"),[](const Json& b){
        Keys(b,{"minimum_m","maximum_m"}); Array(b.at("minimum_m"),3,3); Array(b.at("maximum_m"),3,3); AxisBounds bounds;
        for(std::size_t i=0;i<3;++i) {
            bounds.minimum_m[i]=Number(b.at("minimum_m").at(i)); bounds.maximum_m[i]=Number(b.at("maximum_m").at(i));
            Check(bounds.minimum_m[i]<bounds.maximum_m[i],"Workspace bounds are inverted or empty.");
        } return bounds; });
    auto positive=[](const Json& x){const double n=Number(x); Check(n>0,"A positive limit is required."); return n;};
    l.maximum_path_speed_m_per_s=ReadOptional<double>(v.at("maximum_path_speed_m_per_s"),positive);
    l.maximum_path_acceleration_m_per_s2=ReadOptional<double>(v.at("maximum_path_acceleration_m_per_s2"),positive);
    l.maximum_filament_rate_mm_per_s=ReadOptional<double>(v.at("maximum_filament_rate_mm_per_s"),positive);
    l.maximum_volumetric_flow_mm3_per_s=ReadOptional<double>(v.at("maximum_volumetric_flow_mm3_per_s"),positive);
    auto doubles=[](const Json& array){
        Array(array,8,8); std::array<double,8> values{};
        for(std::size_t i=0;i<8;++i) {values[i]=Number(array.at(i)); Check(values[i]>=0,"Cable limits must be nonnegative.");} return values;};
    l.minimum_cable_length_m=ReadOptional<std::array<double,8>>(v.at("minimum_cable_length_m"),doubles);
    l.maximum_cable_length_m=ReadOptional<std::array<double,8>>(v.at("maximum_cable_length_m"),doubles);
    l.maximum_cable_speed_m_per_s=ReadOptional<std::array<double,8>>(v.at("maximum_cable_speed_m_per_s"),doubles);
    l.maximum_cable_acceleration_m_per_s2=ReadOptional<std::array<double,8>>(v.at("maximum_cable_acceleration_m_per_s2"),doubles);
    auto counts=[](const Json& array){
        Array(array,8,8); std::array<std::int32_t,8> values{};
        for(std::size_t i=0;i<8;++i) {
            const auto& value=array.at(i); Check(value.is_number_integer(),"Count limits require integer literals.");
            if(value.is_number_unsigned()) Check(value.get<std::uint64_t>()<=2147483647ULL,"Count exceeds int32.");
            const auto n=value.get<std::int64_t>(); Check(n>=-2147483648LL&&n<=2147483647LL,"Count exceeds int32.");
            values[i]=static_cast<std::int32_t>(n);
        } return values;};
    l.minimum_motor_count=ReadOptional<std::array<std::int32_t,8>>(v.at("minimum_motor_count"),counts);
    l.maximum_motor_count=ReadOptional<std::array<std::int32_t,8>>(v.at("maximum_motor_count"),counts);
    for(std::size_t i=0;i<8;++i) {
        if(l.maximum_cable_length_m.has_value) Check(l.maximum_cable_length_m.value[i]>0,"Maximum cable lengths must be positive.");
        if(l.maximum_cable_speed_m_per_s.has_value) Check(l.maximum_cable_speed_m_per_s.value[i]>0,"Cable speed limits must be positive.");
        if(l.maximum_cable_acceleration_m_per_s2.has_value) Check(l.maximum_cable_acceleration_m_per_s2.value[i]>0,"Cable acceleration limits must be positive.");
        if(l.minimum_cable_length_m.has_value&&l.maximum_cable_length_m.has_value)
            Check(l.minimum_cable_length_m.value[i]<l.maximum_cable_length_m.value[i],"Cable length bounds are inverted.");
        if(l.minimum_motor_count.has_value&&l.maximum_motor_count.has_value)
            Check(l.minimum_motor_count.value[i]<l.maximum_motor_count.value[i],"Motor count bounds are inverted.");
    }
    return l;
}
Json WriteIdentity(const ModelIdentity& identity) {
    return {{"robot_config_sha256",identity.robot_config_sha256},{"calibration_sha256",identity.calibration_sha256},
        {"rail_state_sha256",identity.rail_state_sha256},{"motion_limits_sha256",identity.motion_limits_sha256}};
}
ModelIdentity ReadIdentity(const Json& v) {
    Keys(v,{"robot_config_sha256","calibration_sha256","rail_state_sha256","motion_limits_sha256"}); ModelIdentity result;
    result.robot_config_sha256=String(v.at("robot_config_sha256")); result.calibration_sha256=String(v.at("calibration_sha256"));
    result.rail_state_sha256=String(v.at("rail_state_sha256")); result.motion_limits_sha256=String(v.at("motion_limits_sha256"));
    Hash(result.robot_config_sha256); Hash(result.calibration_sha256); Hash(result.rail_state_sha256); Hash(result.motion_limits_sha256); return result;
}
Json WriteGeometry(const MotionGeometry& g) {return {{"frame_id",g.frame_id},{"start",WritePose(g.start)},{"end",WritePose(g.end)}};}
MotionGeometry ReadGeometry(const Json& v) {
    Keys(v,{"frame_id","start","end"}); MotionGeometry result;
    result.frame_id=String(v.at("frame_id")); result.start=ReadPose(v.at("start")); result.end=ReadPose(v.at("end")); return result;
}
Json WriteTiming(const OperationTiming& t) {
    Json knots=Json::array();
    for(const auto& k:t.profile) knots.push_back({{"offset_s",k.offset_s},{"progress",k.progress},
        {"path_speed_m_per_s",OptionalNumber(k.path_speed_m_per_s)},{"filament_offset_mm",k.filament_offset_mm},{"filament_rate_mm_per_s",k.filament_rate_mm_per_s}});
    return {{"interpolation",t.interpolation},{"epoch_id",t.epoch_id},{"start_s",t.start_s},{"duration_s",t.duration_s},{"profile",std::move(knots)}};
}
OperationTiming ReadTiming(const Json& v) {
    Keys(v,{"interpolation","epoch_id","start_s","duration_s","profile"}); OperationTiming t;
    t.interpolation=String(v.at("interpolation")); t.epoch_id=Index(v.at("epoch_id"));
    t.start_s=Number(v.at("start_s")); t.duration_s=Number(v.at("duration_s")); Array(v.at("profile"),0,50000);
    for(const auto& k:v.at("profile")) {
        Keys(k,{"offset_s","progress","path_speed_m_per_s","filament_offset_mm","filament_rate_mm_per_s"}); ProfileKnot knot;
        knot.offset_s=Number(k.at("offset_s")); knot.progress=Number(k.at("progress")); knot.path_speed_m_per_s=ReadOptionalNumber(k.at("path_speed_m_per_s"));
        knot.filament_offset_mm=Number(k.at("filament_offset_mm")); knot.filament_rate_mm_per_s=Number(k.at("filament_rate_mm_per_s")); t.profile.push_back(knot);
    } return t;
}
Json WriteLocation(const SourceLocation& s) {
    return {{"first_line",s.first_line}, {"last_line",s.last_line}, {"synthetic",s.synthetic},
        {"command",s.command}, {"layer_index",s.layer_index.has_value ? Json(s.layer_index.value) : Json(nullptr)}};
}
SourceLocation ReadLocation(const Json& value) {
    Keys(value, {"first_line","last_line","synthetic","command","layer_index"});
    SourceLocation s;
    s.first_line = Index(value.at("first_line")); s.last_line = Index(value.at("last_line"));
    s.synthetic = Boolean(value.at("synthetic")); s.command = String(value.at("command"), 0, 4096);
    s.layer_index = ReadOptional<std::uint64_t>(value.at("layer_index"), [](const Json& v) { return Index(v); });
    Location(s); return s;
}
Json WriteDiagnostics(const std::vector<Diagnostic>& values) {
    Json array = Json::array();
    for(const auto& d : values) array.push_back({{"severity",Name(d.severity)}, {"code",Name(d.code)}, {"message",d.message},
        {"operation_id",d.operation_id.has_value ? Json(d.operation_id.value) : Json(nullptr)},
        {"source",d.source.has_value ? WriteLocation(d.source.value) : Json(nullptr)}});
    return array;
}
std::vector<Diagnostic> ReadDiagnostics(const Json& value, std::size_t& total) {
    Array(value, 0, 10000);
    Check(total <= 10000 - value.size(), "Diagnostic limit exceeded."); total += value.size();
    std::vector<Diagnostic> result;
    for(const auto& v : value) {
        Keys(v, {"severity","code","message","operation_id","source"});
        Diagnostic d;
        d.severity = ReadEnum<Severity>(v.at("severity")); d.code = ReadEnum<DiagnosticCode>(v.at("code")); d.message = String(v.at("message"),1,4096);
        d.operation_id = ReadOptional<std::uint64_t>(v.at("operation_id"), [](const Json& item) { return Index(item); });
        d.source = ReadOptional<SourceLocation>(v.at("source"), ReadLocation);
        result.push_back(std::move(d));
    }
    return result;
}
Json WriteMotion(const MotionOperation& m) {
    Json spans = Json::array();
    for(const auto& s : m.extrusion_spans) spans.push_back({{"start_fraction",s.start_fraction}, {"end_fraction",s.end_fraction},
        {"purpose",Name(s.purpose)}, {"delta_filament_mm",s.delta_filament_mm}});
    return {{"intent",Name(m.intent)}, {"nozzle",WriteGeometry(m.nozzle)},
        {"end_effector",m.end_effector.has_value?WriteGeometry(m.end_effector.value):Json(nullptr)}, {"tool_id",m.tool_id}, {"delta_filament_mm",m.delta_filament_mm},
        {"logical_e_start_mm",m.logical_e_start_mm}, {"logical_e_end_mm",m.logical_e_end_mm},
        {"filament_position_start_mm",m.filament_position_start_mm}, {"filament_position_end_mm",m.filament_position_end_mm},
        {"path_feedrate_m_per_s",OptionalNumber(m.path_feedrate_m_per_s)}, {"filament_feedrate_mm_per_s",OptionalNumber(m.filament_feedrate_mm_per_s)},
        {"extrusion_spans",std::move(spans)}};
}
MotionOperation ReadMotion(const Json& v) {
    Keys(v, {"intent","nozzle","end_effector","tool_id","delta_filament_mm","logical_e_start_mm","logical_e_end_mm",
        "filament_position_start_mm","filament_position_end_mm","path_feedrate_m_per_s","filament_feedrate_mm_per_s","extrusion_spans"});
    MotionOperation m;
    m.intent = ReadEnum<MotionIntent>(v.at("intent"));
    m.nozzle=ReadGeometry(v.at("nozzle")); m.end_effector=ReadOptional<MotionGeometry>(v.at("end_effector"),ReadGeometry);
    m.tool_id = static_cast<std::uint32_t>(Index(v.at("tool_id"),0));
    m.delta_filament_mm = Number(v.at("delta_filament_mm"));
    m.logical_e_start_mm = Number(v.at("logical_e_start_mm")); m.logical_e_end_mm = Number(v.at("logical_e_end_mm"));
    m.filament_position_start_mm = Number(v.at("filament_position_start_mm")); m.filament_position_end_mm = Number(v.at("filament_position_end_mm"));
    m.path_feedrate_m_per_s = ReadOptionalNumber(v.at("path_feedrate_m_per_s")); m.filament_feedrate_mm_per_s = ReadOptionalNumber(v.at("filament_feedrate_mm_per_s"));
    Array(v.at("extrusion_spans"),1,2);
    for(const auto& span : v.at("extrusion_spans")) {
        Keys(span, {"start_fraction","end_fraction","purpose","delta_filament_mm"}); ExtrusionSpan s;
        s.start_fraction = Number(span.at("start_fraction")); s.end_fraction = Number(span.at("end_fraction"));
        s.purpose = ReadEnum<ExtrusionPurpose>(span.at("purpose")); s.delta_filament_mm = Number(span.at("delta_filament_mm"));
        m.extrusion_spans.push_back(s);
    }
    return m;
}
Json WriteOperation(const Operation& op) {
    Json temperature = nullptr;
    if(op.temperature.has_value) {
        const auto& t = op.temperature.value;
        temperature = {{"heater",Name(t.heater)}, {"channel",t.channel}, {"target_deg_c",t.target_deg_c},
            {"wait_mode",t.wait_mode.has_value ? Json(Name(t.wait_mode.value)) : Json(nullptr)},
            {"tolerance_deg_c",OptionalNumber(t.tolerance_deg_c)}, {"timeout_s",OptionalNumber(t.timeout_s)}};
    }
    return {{"id",op.id}, {"kind",Name(op.kind)}, {"source",WriteLocation(op.source)}, {"timing",op.timing.has_value?WriteTiming(op.timing.value):Json(nullptr)},
        {"motion",op.motion.has_value ? WriteMotion(op.motion.value) : Json(nullptr)},
        {"dwell",op.dwell.has_value ? Json{{"duration_s",op.dwell.value.duration_s}} : Json(nullptr)},
        {"temperature",std::move(temperature)},
        {"fan",op.fan.has_value ? Json{{"channel",op.fan.value.channel},{"fraction",op.fan.value.fraction}} : Json(nullptr)},
        {"barrier",op.barrier.has_value ? Json{{"reason",Name(op.barrier.value.reason)}} : Json(nullptr)}};
}
Operation ReadOperation(const Json& v) {
    Keys(v, {"id","kind","source","timing","motion","dwell","temperature","fan","barrier"});
    Operation op;
    op.id = Index(v.at("id")); op.kind = ReadEnum<OperationKind>(v.at("kind")); op.source = ReadLocation(v.at("source"));
    op.timing=ReadOptional<OperationTiming>(v.at("timing"),ReadTiming);
    op.motion = ReadOptional<MotionOperation>(v.at("motion"), ReadMotion);
    op.dwell = ReadOptional<DwellOperation>(v.at("dwell"), [](const Json& d) {
        Keys(d, {"duration_s"}); DwellOperation payload; payload.duration_s = Number(d.at("duration_s")); return payload; });
    op.temperature = ReadOptional<TemperatureOperation>(v.at("temperature"), [](const Json& t) {
        Keys(t, {"heater","channel","target_deg_c","wait_mode","tolerance_deg_c","timeout_s"});
        TemperatureOperation payload;
        payload.tolerance_deg_c=ReadOptionalNumber(t.at("tolerance_deg_c")); payload.timeout_s=ReadOptionalNumber(t.at("timeout_s"));
        payload.heater = ReadEnum<HeaterKind>(t.at("heater")); payload.channel = static_cast<std::uint32_t>(Index(t.at("channel"),0));
        payload.target_deg_c = Number(t.at("target_deg_c"));
        payload.wait_mode = ReadOptional<TemperatureWaitMode>(t.at("wait_mode"), ReadEnum<TemperatureWaitMode>);
        return payload; });
    op.fan = ReadOptional<FanOperation>(v.at("fan"), [](const Json& f) {
        Keys(f, {"channel","fraction"}); FanOperation payload; payload.channel = static_cast<std::uint32_t>(Index(f.at("channel"),0));
        payload.fraction = Number(f.at("fraction")); return payload; });
    op.barrier = ReadOptional<BarrierOperation>(v.at("barrier"), [](const Json& b) {
        Keys(b, {"reason"}); BarrierOperation payload; payload.reason = ReadEnum<BarrierReason>(b.at("reason")); return payload; });
    return op;
}
Json WriteJob(const PrintJob& job) {
    Json producer = nullptr;
    if(job.source.producer.has_value) producer = {{"name",job.source.producer.value.name},{"version",job.source.producer.value.version}};
    Json operations = Json::array(), checks = Json::array();
    for(const auto& operation : job.operations) operations.push_back(WriteOperation(operation));
    for(const auto& check : job.validation.checks) checks.push_back({{"kind",Name(check.kind)}, {"status",Name(check.status)}, {"diagnostics",WriteDiagnostics(check.diagnostics)}});
    return {{"schema_version",job.schema_version}, {"job_id",job.job_id}, {"stage",Name(job.stage)},
        {"source",{{"file_name",job.source.file_name},{"sha256",job.source.sha256},{"dialect_id",job.source.dialect_id},
            {"producer",std::move(producer)},{"profile_sha256",job.source.profile_sha256.has_value ? Json(job.source.profile_sha256.value) : Json(nullptr)}}},
        {"units",{{"position",job.units.position},{"orientation",job.units.orientation},{"filament",job.units.filament},{"time",job.units.time},{"temperature",job.units.temperature}}},
        {"model_identity",job.model_identity.has_value?WriteIdentity(job.model_identity.value):Json(nullptr)},
        {"frames",WriteFrames(job.frames)},
        {"operations",std::move(operations)},
        {"validation",{{"checks",std::move(checks)},{"diagnostics",WriteDiagnostics(job.validation.diagnostics)},{"hardware_ready",false}}}};
}
PrintJob ReadJob(const Json& root) {
    Keys(root, {"schema_version","job_id","stage","source","units","model_identity","frames","operations","validation"});
    Check(Index(root.at("schema_version")) == kPrintJobSchemaVersion, "Unsupported schema version.");
    PrintJob job; job.job_id=String(root.at("job_id")); job.stage=ReadEnum<PlanningStage>(root.at("stage"));
    job.model_identity=ReadOptional<ModelIdentity>(root.at("model_identity"),ReadIdentity);
    const auto& s = root.at("source"); Keys(s, {"file_name","sha256","dialect_id","producer","profile_sha256"});
    job.source.file_name = String(s.at("file_name"),1,1024); job.source.sha256 = String(s.at("sha256"));
    job.source.dialect_id = String(s.at("dialect_id"));
    job.source.profile_sha256 = ReadOptional<std::string>(s.at("profile_sha256"), [](const Json& v) { return String(v); });
    job.source.producer = ReadOptional<ProducerInfo>(s.at("producer"), [](const Json& p) {
        Keys(p, {"name","version"}); ProducerInfo info; info.name = String(p.at("name")); info.version = String(p.at("version")); return info; });
    const auto& u = root.at("units"); Keys(u, {"position","orientation","filament","time","temperature"});
    job.units.position = String(u.at("position")); job.units.orientation = String(u.at("orientation"));
    job.units.filament = String(u.at("filament")); job.units.time = String(u.at("time")); job.units.temperature = String(u.at("temperature"));
    job.frames=ReadFrames(root.at("frames"));
    Array(root.at("operations"),2,1000000);
    std::size_t total_knots=0;
    for(const auto& operation:root.at("operations")) if(operation.is_object()&&operation.contains("timing")&&operation.at("timing").is_object()
        &&operation.at("timing").contains("profile")&&operation.at("timing").at("profile").is_array()) {
        const auto count=operation.at("timing").at("profile").size();
        Check(count<=200000-total_knots,"Job profile knot limit exceeded."); total_knots+=count;
    }
    for(const auto& operation : root.at("operations")) job.operations.push_back(ReadOperation(operation));
    const auto& v = root.at("validation"); Keys(v, {"checks","diagnostics","hardware_ready"});
    job.validation.hardware_ready = Boolean(v.at("hardware_ready"));
    std::size_t diagnostic_count = 0;
    job.validation.diagnostics = ReadDiagnostics(v.at("diagnostics"),diagnostic_count);
    Array(v.at("checks"),EnumValues<ValidationCheckKind>().size(),EnumValues<ValidationCheckKind>().size());
    for(const auto& c : v.at("checks")) {
        Keys(c, {"kind","status","diagnostics"}); ValidationCheck check;
        check.kind = ReadEnum<ValidationCheckKind>(c.at("kind")); check.status = ReadEnum<ValidationStatus>(c.at("status"));
        check.diagnostics = ReadDiagnostics(c.at("diagnostics"),diagnostic_count); job.validation.checks.push_back(std::move(check));
    }
    CheckJob(job); return job;
}

std::uint32_t RotateRight(std::uint32_t value, unsigned bits) { return (value >> bits) | (value << (32 - bits)); }
std::string Sha256(const std::string& bytes) {
    Check(bytes.size() <= kMaximumJsonBytes, "Hash input byte limit exceeded.", DiagnosticCode::limit_exceeded);
    static const std::uint32_t constants[64] = {
        0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
        0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
        0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
        0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
        0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
        0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
        0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
        0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u};
    std::uint32_t state[8] = {0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u};
    const std::size_t padded = ((bytes.size() + 9 + 63) / 64) * 64;
    const auto bit_length = static_cast<std::uint64_t>(bytes.size()) * 8;
    for(std::size_t block = 0; block < padded; block += 64) {
        std::uint32_t words[64]{};
        for(std::size_t i = 0; i < 64; ++i) {
            const auto at = block + i;
            unsigned char byte = at < bytes.size() ? static_cast<unsigned char>(bytes[at]) : at == bytes.size() ? 0x80 : 0;
            if(at >= padded - 8) byte = static_cast<unsigned char>(bit_length >> (8 * (padded - 1 - at)));
            words[i / 4] |= static_cast<std::uint32_t>(byte) << (24 - 8 * (i % 4));
        }
        for(std::size_t i = 16; i < 64; ++i) {
            const auto x = words[i-15], y = words[i-2];
            words[i] = words[i-16] + (RotateRight(x,7) ^ RotateRight(x,18) ^ (x >> 3))
                + words[i-7] + (RotateRight(y,17) ^ RotateRight(y,19) ^ (y >> 10));
        }
        auto a=state[0], b=state[1], c=state[2], d=state[3], e=state[4], f=state[5], g=state[6], h=state[7];
        for(std::size_t i = 0; i < 64; ++i) {
            const auto first = h + (RotateRight(e,6) ^ RotateRight(e,11) ^ RotateRight(e,25))
                + ((e & f) ^ (~e & g)) + constants[i] + words[i];
            const auto second = (RotateRight(a,2) ^ RotateRight(a,13) ^ RotateRight(a,22)) + ((a & b) ^ (a & c) ^ (b & c));
            h=g; g=f; f=e; e=d+first; d=c; c=b; b=a; a=first+second;
        }
        state[0]+=a; state[1]+=b; state[2]+=c; state[3]+=d; state[4]+=e; state[5]+=f; state[6]+=g; state[7]+=h;
    }
    static const char hex[] = "0123456789abcdef";
    std::string digest; digest.reserve(64);
    for(auto word : state) for(int shift = 28; shift >= 0; shift -= 4) digest += hex[(word >> shift) & 15];
    return digest;
}

} // namespace

Result<std::string> ComputeSourceSha256(const std::string& bytes) {
    return Attempt<std::string>([&] { return Sha256(bytes); });
}
Result<ValidationReport> ValidateImportedJob(const PrintJob& job) {
    return Attempt<ValidationReport>([&] {
        Check(job.stage==PlanningStage::imported,"An imported job is required."); CheckJob(job); return job.validation; });
}
Result<ValidationReport> ValidatePlanningArtifact(const PrintJob& job) {
    return Attempt<ValidationReport>([&] {CheckJob(job); return job.validation;});
}
Result<MotionLimits> DeserializeMotionLimits(const std::string& json_text) {
    return Attempt<MotionLimits>([&] {return ReadLimits(ParseJson(json_text));});
}
Result<FrameRegistration> DeserializeFrameRegistration(const std::string& json_text) {
    return Attempt<FrameRegistration>([&] {return ReadFrames(ParseJson(json_text));});
}
Result<std::string> SerializeMotionLimits(const MotionLimits& limits) {
    return Attempt<std::string>([&] {auto value=WriteLimits(limits); ReadLimits(value); return value.dump();});
}
Result<std::string> SerializeFrameRegistration(const FrameRegistration& frames) {
    return Attempt<std::string>([&] {
        Check(frames.status==RegistrationStatus::unverified || frames.status==RegistrationStatus::verified,"Invalid registration enum.");
        auto value=WriteFrames(frames); ReadFrames(value); return value.dump();});
}
Result<PlanningOptions> DeserializePlanningOptions(const std::string& json_text) {
    return Attempt<PlanningOptions>([&] {
        const auto root=ParseJson(json_text); Keys(root,{"frames","planning"}); PlanningOptions options;
        options.frames=ReadFrames(root.at("frames")); const auto& v=root.at("planning");
        Keys(v,{"fixed_orientation","fixed_rails","sample_period_s","path_tolerance_m","filament_diameter_mm",
            "temperature_wait_policy","limits","maximum_samples","maximum_samples_per_motion"});
        options.fixed_orientation=Boolean(v.at("fixed_orientation")); options.fixed_rails=Boolean(v.at("fixed_rails"));
        auto positive=[](const Json& value){const double n=Number(value); Check(n>0,"Planning values must be positive."); return n;};
        options.sample_period_s=ReadOptional<double>(v.at("sample_period_s"),positive);
        options.path_tolerance_m=ReadOptional<double>(v.at("path_tolerance_m"),positive);
        options.filament_diameter_mm=ReadOptional<double>(v.at("filament_diameter_mm"),positive);
        options.maximum_samples=static_cast<std::size_t>(Index(v.at("maximum_samples"),200000));
        options.maximum_samples_per_motion=static_cast<std::size_t>(Index(v.at("maximum_samples_per_motion"),50000));
        Check(options.maximum_samples>=5 && options.maximum_samples_per_motion>=5
            && options.maximum_samples_per_motion<=options.maximum_samples,"Planning sample bounds are invalid.");
        options.limits=ReadLimits(v.at("limits"));
        const auto& wait=v.at("temperature_wait_policy"); Keys(wait,{"hotend","bed"});
        auto limits=[&](const Json& value){
            Keys(value,{"tolerance_deg_c","timeout_s"}); TemperatureWaitLimits policy;
            policy.tolerance_deg_c=ReadOptional<double>(value.at("tolerance_deg_c"),positive);
            policy.timeout_s=ReadOptional<double>(value.at("timeout_s"),positive);
            Check(!policy.timeout_s.has_value||policy.timeout_s.value<=86400,"Thermal wait timeout exceeds one day.");
            Check(policy.tolerance_deg_c.has_value==policy.timeout_s.has_value,"A thermal policy needs both tolerance and timeout.");
            return policy;};
        options.temperature_wait_policy.hotend=limits(wait.at("hotend")); options.temperature_wait_policy.bed=limits(wait.at("bed"));
        return options;
    });
}
Result<std::string> SerializePrintJob(const PrintJob& job) {
    return Attempt<std::string>([&] {
        CheckJob(job);
        auto text = WriteJob(job).dump(2) + "\n";
        Check(text.size() <= kMaximumJsonBytes, "Serialized job byte limit exceeded.", DiagnosticCode::limit_exceeded);
        return text;
    });
}
Result<PrintJob> DeserializePrintJob(const std::string& json_text) {
    return Attempt<PrintJob>([&] { return ReadJob(ParseJson(json_text)); });
}
Result<GCodeImportOptions> DeserializeImportOptions(const std::string& json_text) {
    return Attempt<GCodeImportOptions>([&] {
        const auto v = ParseJson(json_text);
        Keys(v, {"dialect_id","initial_state","maximum_input_bytes","maximum_line_bytes","maximum_operations","maximum_diagnostics"});
        GCodeImportOptions options; options.dialect_id = String(v.at("dialect_id"));
        Check(options.dialect_id == "marlin_restricted_v1", "Unsupported dialect.");
        options.maximum_input_bytes = static_cast<std::size_t>(Index(v.at("maximum_input_bytes"),kMaximumJsonBytes));
        options.maximum_line_bytes = static_cast<std::size_t>(Index(v.at("maximum_line_bytes"),4096));
        options.maximum_operations = static_cast<std::size_t>(Index(v.at("maximum_operations"),1000000));
        options.maximum_diagnostics = static_cast<std::size_t>(Index(v.at("maximum_diagnostics"),10000));
        Check(options.maximum_input_bytes && options.maximum_line_bytes && options.maximum_operations && options.maximum_diagnostics, "Import resource limits must be positive.");
        const auto& s = v.at("initial_state");
        Keys(s, {"units","xyz_mode","extrusion_mode","nozzle_pose_in_bed","logical_e_mm","filament_position_mm","feedrate_source_units_per_min"});
        auto& initial = options.initial_state;
        initial.units = ReadEnum<SourceLengthUnit>(s.at("units")); initial.xyz_mode = ReadEnum<PositionMode>(s.at("xyz_mode"));
        initial.extrusion_mode = ReadEnum<PositionMode>(s.at("extrusion_mode"));
        initial.nozzle_pose_in_bed = ReadOptional<CartesianPose>(s.at("nozzle_pose_in_bed"), ReadPose);
        initial.logical_e_mm = ReadOptionalNumber(s.at("logical_e_mm")); initial.filament_position_mm = ReadOptionalNumber(s.at("filament_position_mm"));
        initial.feedrate_source_units_per_min = ReadOptionalNumber(s.at("feedrate_source_units_per_min"));
        Check(!initial.feedrate_source_units_per_min.has_value || (initial.units != SourceLengthUnit::unspecified
            && initial.feedrate_source_units_per_min.value > 0), "Initial feedrate requires known units and a positive value.");
        return options;
    });
}

} // namespace cdpr_slicer
