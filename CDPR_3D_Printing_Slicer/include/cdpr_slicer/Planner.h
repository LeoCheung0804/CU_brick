#ifndef CDPR_SLICER_PLANNER_H
#define CDPR_SLICER_PLANNER_H

#include "cdpr_slicer/Toolpath.h"
#include "CableKinematics.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace cdpr_slicer {

enum class SourceLengthUnit { unspecified, millimetres, inches };
enum class PositionMode { unspecified, absolute, relative };

struct InitialModalState {
    SourceLengthUnit units{SourceLengthUnit::unspecified};
    PositionMode xyz_mode{PositionMode::unspecified};
    PositionMode extrusion_mode{PositionMode::unspecified};
    OptionalValue<CartesianPose> nozzle_pose_in_bed;
    OptionalValue<double> logical_e_mm;
    OptionalValue<double> filament_position_mm;
    OptionalValue<double> feedrate_source_units_per_min;
};

struct GCodeImportOptions {
    std::string dialect_id{"marlin_restricted_v1"};
    InitialModalState initial_state;
    std::size_t maximum_input_bytes{64u * 1024u * 1024u};
    std::size_t maximum_line_bytes{4096};
    std::size_t maximum_operations{1000000};
    std::size_t maximum_diagnostics{10000};
};

struct GCodeParseRequest {
    std::string text;
    SourceInfo source;
    GCodeImportOptions options;
};

struct AxisBounds {
    std::array<double, 3> minimum_m{};
    std::array<double, 3> maximum_m{};
};

// Unknown mechanical limits remain absent; no configuration defaults imply safety.
struct MotionLimits {
    OptionalValue<AxisBounds> workspace;
    OptionalValue<double> maximum_path_speed_m_per_s;
    OptionalValue<double> maximum_path_acceleration_m_per_s2;
    OptionalValue<std::array<double, cu_brick::kMaximumCableCount>> minimum_cable_length_m;
    OptionalValue<std::array<double, cu_brick::kMaximumCableCount>> maximum_cable_length_m;
    OptionalValue<std::array<double, cu_brick::kMaximumCableCount>> maximum_cable_speed_m_per_s;
    OptionalValue<std::array<double, cu_brick::kMaximumCableCount>> maximum_cable_acceleration_m_per_s2;
    OptionalValue<std::array<std::int32_t, cu_brick::kMaximumCableCount>> minimum_motor_count;
    OptionalValue<std::array<std::int32_t, cu_brick::kMaximumCableCount>> maximum_motor_count;
    OptionalValue<double> maximum_filament_rate_mm_per_s;
    OptionalValue<double> maximum_volumetric_flow_mm3_per_s;
};

struct RobotModelSnapshot {
    RobotModelSnapshot() {
        // Eigen's default fixed-size vectors are uninitialized; an incomplete
        // public snapshot must instead contain deterministic invalid geometry.
        for(std::size_t i=0;i<cu_brick::kMaximumCableCount;++i) {
            kinematics.frameAttachments[i].setZero();
            kinematics.endEffectorAttachments[i].setZero();
            kinematics.pulleyAxes[i].setZero();
        }
    }
    ModelIdentity identity;
    cu_brick::CableKinematicsConfig kinematics;
    std::size_t rail_count{0};
    std::array<double, cu_brick::kMaximumRailCount> rail_offsets_m{};
    std::array<double, cu_brick::kMaximumCableCount> cable_scale_counts_per_m{};
    MotionLimits limits;
    FrameRegistration frames;
    bool has_calibration{false};
    bool has_motion_limits{false};
    // Internal consistency seal over the decoded snapshot, not a signature.
    std::string snapshot_sha256;
};

struct ModelLoadRequest {
    std::string robot_config_path;
    OptionalValue<std::string> expected_robot_config_sha256;
    OptionalValue<std::string> calibration_manifest_path;
    OptionalValue<std::string> motion_limits_manifest_path;
    OptionalValue<std::array<double, cu_brick::kMaximumRailCount>> rail_offsets_m;
    OptionalValue<std::array<double, cu_brick::kMaximumRailCount>> starting_rail_offsets_m;
};

struct TemperatureWaitLimits {
    OptionalValue<double> tolerance_deg_c;
    OptionalValue<double> timeout_s;
};

struct TemperatureWaitPolicy {
    TemperatureWaitLimits hotend;
    TemperatureWaitLimits bed;
};

struct PlanningOptions {
    OptionalValue<ModelIdentity> model_identity;
    FrameRegistration frames;
    MotionLimits limits;
    TemperatureWaitPolicy temperature_wait_policy;
    OptionalValue<double> sample_period_s;
    OptionalValue<double> path_tolerance_m;
    OptionalValue<double> filament_diameter_mm;
    bool fixed_orientation{true};
    bool fixed_rails{true};
    std::size_t maximum_samples{200000};
    std::size_t maximum_samples_per_motion{50000};
};

// Offline mathematical interfaces only. None constructs or connects a Robot.
Result<PrintJob> ParseGCode(const GCodeParseRequest& request);
// Exact byte digest, bounded to 64 MiB; not a signature or authorization.
Result<std::string> ComputeSourceSha256(const std::string& bytes);
// Imported-stage internal consistency only; no physical/planning checks.
Result<ValidationReport> ValidateImportedJob(const PrintJob& job);
// Strictly decode the "import" object from a planner configuration.
Result<GCodeImportOptions> DeserializeImportOptions(const std::string& json_text);
Result<RobotModelSnapshot> LoadRobotModel(const ModelLoadRequest& request);
Result<ValidationReport> ValidateModelSnapshot(const RobotModelSnapshot& model);
Result<PlanningOptions> BindPlanningOptions(
    const PlanningOptions& options, const RobotModelSnapshot& model);
Result<MotionLimits> DeserializeMotionLimits(const std::string& json_text);
Result<FrameRegistration> DeserializeFrameRegistration(const std::string& json_text);
Result<std::string> SerializeMotionLimits(const MotionLimits& limits);
Result<std::string> SerializeFrameRegistration(const FrameRegistration& frames);
// Decode {"frames": ..., "planning": ...}; paths are resolved by the caller.
Result<PlanningOptions> DeserializePlanningOptions(const std::string& json_text);
Result<ValidationReport> ValidatePlanningArtifact(const PrintJob& job);
Result<CartesianPose> TransformNozzleToEndEffector(
    const CartesianPose& nozzle_in_bed, const FrameRegistration& frames);
Result<std::array<double, 6>> ToLegacyPose(const CartesianPose& pose);
Result<PrintJob> TransformToolpath(const PrintJob& job, const PlanningOptions& options);

// Calls the existing ComputeCableLengths implementation after domain checks.
// Pose order here deliberately matches Robot: XYZ meters, then roll/pitch/yaw radians.
Result<std::vector<double>> ComputeCheckedCableLengths(
    const RobotModelSnapshot& model,
    const std::array<double, 6>& legacy_pose_m_rad);

Result<ValidationReport> ValidateWorkspace(
    const PrintJob& job,
    const RobotModelSnapshot& model,
    const PlanningOptions& options);
Result<PrintJob> CreateGeometricPlan(
    const PrintJob& job, const RobotModelSnapshot& model, const PlanningOptions& options);
Result<PrintJob> PlanTrajectory(
    const PrintJob& job,
    const RobotModelSnapshot& model,
    const PlanningOptions& options);

Result<std::string> SerializePrintJob(const PrintJob& job);
Result<PrintJob> DeserializePrintJob(const std::string& json_text);
Result<std::string> CreatePreviewHtml(const PrintJob& job);
// Adds model-bound kinematic playback; does not alter the plan or its checks.
Result<std::string> CreatePreviewHtml(const PrintJob& job, const RobotModelSnapshot& model);

} // namespace cdpr_slicer

#endif
