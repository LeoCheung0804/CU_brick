#ifndef CDPR_SLICER_EXECUTION_H
#define CDPR_SLICER_EXECUTION_H

#include "cdpr_slicer/Toolpath.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cdpr_slicer {

// Hardware-neutral contracts only. There are no concrete device implementations.
enum class SessionState {
    disconnected, not_ready, ready, armed, running, pausing, paused, cancelling,
    cancelled, completed, faulted, emergency_stopped, recovery_required
};
enum class PoseQuality { unknown, commanded_only, encoder_estimate, externally_measured };
enum class CommandDisposition { rejected, accepted, indeterminate, faulted };
enum class ClockMode { unavailable, host_only, shared_clock, shared_trigger };
enum class StopReason { pause, cancel, deadline_miss, communication_loss, device_fault, emergency };

struct MotionCapabilities {
    std::string backend_id;
    std::string backend_version;
    ClockMode clock_mode{ClockMode::unavailable};
    bool scheduled_motion{false};
    bool coordinated_start{false};
    bool queue_status{false};
    bool completion_status{false};
    bool following_feedback{false};
    bool controlled_stop{false};
    OptionalValue<double> clock_error_bound_s;
    OptionalValue<double> minimum_sample_period_s;
    OptionalValue<std::size_t> queue_capacity;
};

struct MotionSnapshot {
    ModelIdentity model_identity;
    PoseQuality pose_quality{PoseQuality::unknown};
    OptionalValue<CartesianPose> commanded_pose;
    OptionalValue<CartesianPose> measured_or_estimated_pose;
    OptionalValue<double> feedback_age_s;
    OptionalValue<bool> stationary;
    std::vector<std::int32_t> measured_motor_counts;
    OptionalValue<std::array<double, 4>> rail_offsets_m;
    OptionalValue<std::array<double, 4>> starting_rail_offsets_m;
    OptionalValue<double> maximum_following_error_m;
    bool registration_verified{false};
};

struct SessionPreconditions {
    std::string job_id;
    ModelIdentity expected_model_identity;
    FrameRegistration expected_frames;
    OptionalValue<CartesianPose> expected_start_pose;
    OptionalValue<double> start_pose_tolerance_m;
    OptionalValue<double> start_orientation_tolerance_rad;
    OptionalValue<double> maximum_feedback_age_s;
    OptionalValue<double> maximum_following_error_m;
    OptionalValue<double> maximum_orientation_error_rad;
    OptionalValue<double> maximum_clock_error_s;
    OptionalValue<double> maximum_start_skew_s;
    bool require_verified_registration{true};
    bool require_exclusive_ownership{true};
    bool require_fixed_rails{true};
};

struct SessionToken {
    std::string session_id;
    std::string job_id;
};

struct ExecutionKey {
    std::string session_id;
    std::string job_id;
    std::uint64_t epoch_id{0};
    std::uint64_t chunk_id{0};
};

struct ExecutionEpoch {
    std::uint64_t epoch_id{0};
    std::string clock_domain_id;
    OptionalValue<double> start_clock_s;
    OptionalValue<std::string> trigger_id;
};

struct TimedPoseSample {
    std::uint64_t sequence{0};
    double time_from_epoch_s{0.0};
    CartesianPose end_effector_target;
};

struct MotionChunk {
    ExecutionKey key;
    ModelIdentity model_identity;
    std::array<double, 4> fixed_rail_offsets_m{};
    std::vector<TimedPoseSample> samples;
};

// success means a readable acknowledgement; disposition still governs acceptance.
// Neither success nor accepted establishes physical movement or arrival.
struct SubmissionReceipt {
    CommandDisposition disposition{CommandDisposition::rejected};
    OptionalValue<std::uint64_t> last_accepted_sequence;
    OptionalValue<std::size_t> available_queue_slots;
};

struct ArmReceipt {
    CommandDisposition disposition{CommandDisposition::rejected};
    std::uint64_t epoch_id{0};
    bool armed{false};
};

struct StopReceipt {
    CommandDisposition disposition{CommandDisposition::rejected};
    OptionalValue<bool> stationary_confirmed;
    OptionalValue<bool> extrusion_inhibited_confirmed;
    OptionalValue<bool> brakes_secured_confirmed;
    bool recovery_required{true};
};

struct MotionStatus {
    SessionState state{SessionState::disconnected};
    MotionSnapshot snapshot;
    OptionalValue<std::uint64_t> last_accepted_sequence;
    OptionalValue<std::uint64_t> last_executed_sequence;
    OptionalValue<std::size_t> available_queue_slots;
    std::vector<Diagnostic> diagnostics;
};

class IMotionPort {
public:
    virtual ~IMotionPort() = default;
    virtual Result<MotionCapabilities> Capabilities() const = 0;
    virtual Result<MotionSnapshot> Snapshot() const = 0;
    virtual Result<SessionToken> BeginSession(const SessionPreconditions& expected) = 0;
    virtual Result<SubmissionReceipt> Queue(const SessionToken& session, const MotionChunk& chunk) = 0;
    virtual Result<ArmReceipt> Arm(const SessionToken& session, const ExecutionEpoch& epoch) = 0;
    virtual Result<MotionStatus> Status(const SessionToken& session) const = 0;
    virtual Result<StopReceipt> RequestStop(const SessionToken& session, StopReason reason) = 0;
    // Available for retained resources even if session establishment failed.
    virtual Result<StopReceipt> FaultStop(StopReason reason) = 0;
    virtual Result<SessionState> EndSession(const SessionToken& session) = 0;
};

struct PrintHeadCapabilities {
    std::string firmware_id;
    std::string protocol_version;
    ClockMode clock_mode{ClockMode::unavailable};
    bool scheduled_extrusion{false};
    bool queue_status{false};
    bool cancellation_invalidates_queue{false};
    bool independent_extrusion_inhibit{false};
    bool autonomous_thermal_protection{false};
    bool cold_extrusion_prevention{false};
    bool communication_watchdog{false};
    bool hotend{false};
    bool heated_bed{false};
    bool part_cooling_fan{false};
    OptionalValue<double> clock_error_bound_s;
    OptionalValue<std::size_t> queue_capacity;
};

struct TimedExtrusionKnot {
    std::uint64_t motion_sequence{0};
    double time_from_epoch_s{0.0};
    double filament_position_mm{0.0};
    double filament_rate_mm_per_s{0.0};
};

struct ExtrusionChunk {
    ExecutionKey key;
    std::vector<TimedExtrusionKnot> knots;
};

struct HeaterStatus {
    HeaterKind heater{HeaterKind::hotend};
    std::uint32_t channel{0};
    OptionalValue<double> measured_deg_c;
    OptionalValue<double> target_deg_c;
    OptionalValue<double> measurement_age_s;
    bool ready{false};
};

struct PrintHeadStatus {
    SessionState state{SessionState::disconnected};
    OptionalValue<bool> extrusion_inhibited;
    OptionalValue<std::uint64_t> last_accepted_sequence;
    OptionalValue<std::uint64_t> last_executed_sequence;
    OptionalValue<std::size_t> available_queue_slots;
    std::vector<HeaterStatus> heaters;
    std::vector<Diagnostic> diagnostics;
};

class IPrintHeadPort {
public:
    virtual ~IPrintHeadPort() = default;
    virtual Result<PrintHeadCapabilities> Capabilities() const = 0;
    virtual Result<SessionToken> BeginSession(const SessionPreconditions& expected) = 0;
    virtual Result<SubmissionReceipt> SetTemperature(const SessionToken& session, const TemperatureOperation& target) = 0;
    virtual Result<SubmissionReceipt> SetFan(const SessionToken& session, const FanOperation& target) = 0;
    virtual Result<SubmissionReceipt> Queue(const SessionToken& session, const ExtrusionChunk& chunk) = 0;
    virtual Result<ArmReceipt> Arm(const SessionToken& session, const ExecutionEpoch& epoch) = 0;
    virtual Result<PrintHeadStatus> Status(const SessionToken& session) const = 0;
    // Inhibit must not require a valid session or a successful queue connection.
    virtual Result<StopReceipt> Inhibit(StopReason reason) = 0;
    virtual Result<SessionState> EndSession(const SessionToken& session) = 0;
};

} // namespace cdpr_slicer

#endif
