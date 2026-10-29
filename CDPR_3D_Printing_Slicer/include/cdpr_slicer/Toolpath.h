#ifndef CDPR_SLICER_TOOLPATH_H
#define CDPR_SLICER_TOOLPATH_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace cdpr_slicer {

constexpr std::uint32_t kPrintJobSchemaVersion = 1;

// Serialized as a value or JSON null, not as this wrapper's members.
template<typename T>
struct OptionalValue {
    bool has_value{false};
    T value{};
};

enum class ResultStatus { not_implemented, success, rejected, indeterminate, fault };
enum class Severity { info, warning, error, critical };
enum class DiagnosticCode {
    metadata, state_change, no_op,
    not_implemented, malformed_input, unsupported_command, undefined_modal_state,
    invalid_number, invalid_configuration, invalid_transform, invalid_geometry,
    outside_workspace, limit_exceeded, infeasible_tension, missing_model_data,
    identity_mismatch, invalid_timing, stale_feedback, capability_unavailable,
    queue_fault, communication_fault, thermal_fault, execution_fault
};

struct SourceLocation {
    std::uint64_t first_line{0};
    std::uint64_t last_line{0};
    bool synthetic{false};
    std::string command;
    OptionalValue<std::uint64_t> layer_index;
};

struct Diagnostic {
    Severity severity{Severity::error};
    DiagnosticCode code{DiagnosticCode::not_implemented};
    std::string message;
    OptionalValue<std::uint64_t> operation_id;
    OptionalValue<SourceLocation> source;
};

// value is usable only when status == success. Default construction is not success.
template<typename T>
struct Result {
    ResultStatus status{ResultStatus::not_implemented};
    T value{};
    std::vector<Diagnostic> diagnostics;
};

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

struct CartesianPose {
    std::array<double, 3> position_m{};
    Quaternion orientation_xyzw;
};

// parent_from_child maps child-frame column vectors into the parent frame.
struct RigidTransform {
    std::array<double, 3> translation_m{};
    Quaternion rotation_xyzw;
};

enum class RegistrationStatus { unverified, verified };

struct FrameRegistration {
    std::string world_frame_id{"cdpr_world"};
    std::string bed_frame_id{"print_bed"};
    std::string end_effector_frame_id{"end_effector"};
    std::string nozzle_frame_id{"nozzle_tip"};
    OptionalValue<RigidTransform> world_from_bed;
    OptionalValue<RigidTransform> end_effector_from_nozzle;
    RegistrationStatus status{RegistrationStatus::unverified};
};

// Hash algorithms and byte encodings are specified in docs/architecture.md.
struct ModelIdentity {
    std::string robot_config_sha256;
    std::string calibration_sha256;
    std::string rail_state_sha256;
    std::string motion_limits_sha256;
};

struct ProducerInfo {
    std::string name;
    std::string version;
};

struct SourceInfo {
    std::string file_name;
    std::string sha256;
    std::string dialect_id{"marlin_restricted_v1"};
    OptionalValue<ProducerInfo> producer;
    OptionalValue<std::string> profile_sha256;
};

struct CanonicalUnits {
    std::string position{"m"};
    std::string orientation{"quaternion_xyzw"};
    std::string filament{"mm"};
    std::string time{"s"};
    std::string temperature{"degC"};
};

enum class MotionIntent { unclassified, travel, deposit, retract, recover, prime, mixed };
enum class ExtrusionPurpose { none, deposit, retract, recover, prime };

// Fractions use commanded interpolation progress u, including E-only moves.
// Ordered spans partition [0,1]; their signed filament deltas sum to the move delta.
struct ExtrusionSpan {
    double start_fraction{0.0};
    double end_fraction{1.0};
    ExtrusionPurpose purpose{ExtrusionPurpose::none};
    double delta_filament_mm{0.0};
};

struct MotionGeometry {
    std::string frame_id;
    CartesianPose start;
    CartesianPose end;
};

struct MotionOperation {
    MotionIntent intent{MotionIntent::unclassified};
    MotionGeometry nozzle;
    OptionalValue<MotionGeometry> end_effector;
    std::uint32_t tool_id{0};
    double delta_filament_mm{0.0};
    double logical_e_start_mm{0.0};
    double logical_e_end_mm{0.0};
    double filament_position_start_mm{0.0};
    double filament_position_end_mm{0.0};
    OptionalValue<double> path_feedrate_m_per_s;
    OptionalValue<double> filament_feedrate_mm_per_s;
    std::vector<ExtrusionSpan> extrusion_spans;
};

struct ProfileKnot {
    double offset_s{0.0};
    double progress{0.0};
    OptionalValue<double> path_speed_m_per_s;
    double filament_offset_mm{0.0};
    double filament_rate_mm_per_s{0.0};
};

// Times are relative to an execution epoch, never wall-clock dates.
struct OperationTiming {
    // Motion: quintic_progress_v1. Other events: event_v1.
    std::string interpolation;
    std::uint64_t epoch_id{0};
    double start_s{0.0};
    double duration_s{0.0};
    std::vector<ProfileKnot> profile;
};

struct DwellOperation { double duration_s{0.0}; };
enum class HeaterKind { hotend, bed };
enum class TemperatureWaitMode { heat_only, heat_or_cool };
struct TemperatureOperation {
    HeaterKind heater{HeaterKind::hotend};
    std::uint32_t channel{0};
    double target_deg_c{0.0};
    OptionalValue<TemperatureWaitMode> wait_mode;
    OptionalValue<double> tolerance_deg_c;
    OptionalValue<double> timeout_s;
};
struct FanOperation {
    std::uint32_t channel{0};
    double fraction{0.0};
};
enum class BarrierReason { explicit_sync, pause_boundary, job_end };
struct BarrierOperation { BarrierReason reason{BarrierReason::explicit_sync}; };

enum class OperationKind {
    invalid, motion, dwell, set_temperature, wait_temperature, set_fan, barrier, end_job
};

// Exactly one matching payload is present; end_job has no payload.
struct Operation {
    std::uint64_t id{0};
    OperationKind kind{OperationKind::invalid};
    SourceLocation source;
    OptionalValue<OperationTiming> timing;
    OptionalValue<MotionOperation> motion;
    OptionalValue<DwellOperation> dwell;
    OptionalValue<TemperatureOperation> temperature;
    OptionalValue<FanOperation> fan;
    OptionalValue<BarrierOperation> barrier;
};

enum class ValidationStatus { not_evaluated, passed, failed };
enum class ValidationCheckKind {
    syntax, modal_state, coordinate_transform, geometry, workspace, orientation,
    motor_count, cable_speed, cable_acceleration, collision, singularity, tension,
    execution_timing, feedback, thermal, extrusion_sync, stop_interlocks
};

struct ValidationCheck {
    ValidationCheckKind kind{ValidationCheckKind::syntax};
    ValidationStatus status{ValidationStatus::not_evaluated};
    std::vector<Diagnostic> diagnostics;
};

struct ValidationReport {
    std::vector<ValidationCheck> checks;
    std::vector<Diagnostic> diagnostics;
    // Schema v1 is a planning artifact, not authorization to operate hardware.
    bool hardware_ready{false};
};

enum class PlanningStage { imported, transformed, geometrically_checked, timed, simulated };

struct PrintJob {
    std::uint32_t schema_version{kPrintJobSchemaVersion};
    std::string job_id;
    PlanningStage stage{PlanningStage::imported};
    SourceInfo source;
    CanonicalUnits units;
    OptionalValue<ModelIdentity> model_identity;
    FrameRegistration frames;
    std::vector<Operation> operations;
    ValidationReport validation;
};

} // namespace cdpr_slicer

#endif
