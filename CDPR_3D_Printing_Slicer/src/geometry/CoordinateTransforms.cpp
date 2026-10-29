#include "cdpr_slicer/Planner.h"
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {
void Require(bool condition, const char* message) {
    if(!condition) throw std::invalid_argument(message);
}
Eigen::Quaterniond Rotation(const Quaternion& q) {
    Require(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w)
        && std::abs(q.x) <= 1 && std::abs(q.y) <= 1 && std::abs(q.z) <= 1 && std::abs(q.w) <= 1,
        "Quaternion components must be finite and in [-1,1].");
    Eigen::Quaterniond result(q.w,q.x,q.y,q.z);
    Require(std::abs(result.squaredNorm() - 1) <= 1e-10, "A unit quaternion is required.");
    // Remove only roundoff from an already valid unit quaternion.
    result.normalize(); return result;
}
Eigen::Vector3d Position(const std::array<double,3>& p) {
    for(auto value : p) Require(std::isfinite(value), "Position must be finite.");
    return Eigen::Vector3d(p[0],p[1],p[2]);
}
Eigen::Isometry3d Transform(const RigidTransform& value) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = Rotation(value.rotation_xyzw).toRotationMatrix();
    result.translation() = Position(value.translation_m); return result;
}
template<typename T, typename F> Result<T> Attempt(F action) {
    Result<T> result;
    try { result.value = action(); result.status = ResultStatus::success; }
    catch(const std::invalid_argument& error) {
        result.status = ResultStatus::rejected;
        Diagnostic d; d.code = DiagnosticCode::invalid_transform; d.message = error.what(); result.diagnostics.push_back(d);
    } catch(const std::exception&) {
        result.status = ResultStatus::fault;
        Diagnostic d; d.code = DiagnosticCode::execution_fault; d.message = "Coordinate transformation could not complete."; result.diagnostics.push_back(d);
    }
    return result;
}
void Passed(PrintJob& job, ValidationCheckKind kind) {
    for(auto& check : job.validation.checks) if(check.kind == kind) check.status = ValidationStatus::passed;
}
} // namespace

Result<CartesianPose> TransformNozzleToEndEffector(const CartesianPose& nozzle, const FrameRegistration& frames) {
    return Attempt<CartesianPose>([&] {
        Require(frames.world_from_bed.has_value && frames.end_effector_from_nozzle.has_value
            && frames.status == RegistrationStatus::verified, "Verified, explicit bed and nozzle transforms are required.");
        Eigen::Isometry3d bed_from_nozzle = Eigen::Isometry3d::Identity();
        bed_from_nozzle.linear() = Rotation(nozzle.orientation_xyzw).toRotationMatrix();
        bed_from_nozzle.translation() = Position(nozzle.position_m);
        const Eigen::Isometry3d world_from_ee = Transform(frames.world_from_bed.value)
            * bed_from_nozzle * Transform(frames.end_effector_from_nozzle.value).inverse();
        Require(world_from_ee.matrix().allFinite(), "Transformation arithmetic overflow.");
        Eigen::Quaterniond q(world_from_ee.linear()); q.normalize();
        if(q.w() < 0) q.coeffs() *= -1;
        CartesianPose result;
        for(std::size_t i = 0; i < 3; ++i) result.position_m[i] = world_from_ee.translation()[static_cast<int>(i)];
        result.orientation_xyzw.x = q.x(); result.orientation_xyzw.y = q.y();
        result.orientation_xyzw.z = q.z(); result.orientation_xyzw.w = q.w();
        return result;
    });
}

Result<std::array<double,6>> ToLegacyPose(const CartesianPose& pose) {
    return Attempt<std::array<double,6>>([&] {
        Position(pose.position_m);
        const Eigen::Matrix3d r = Rotation(pose.orientation_xyzw).toRotationMatrix();
        // R = Rx(roll)*Ry(pitch)*Rz(yaw): R02=sin(pitch).
        Require(std::abs(r(0,2)) <= 1, "Rotation matrix lies outside the Euler domain.");
        const double pitch = std::asin(r(0,2));
        Require(std::abs(std::cos(pitch)) > 1e-8, "Legacy XYZ Euler conversion is near gimbal lock.");
        const double roll = std::atan2(-r(1,2),r(2,2));
        const double yaw = std::atan2(-r(0,1),r(0,0));
        const Eigen::Matrix3d rebuilt = (Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())).toRotationMatrix();
        Require((rebuilt-r).cwiseAbs().maxCoeff() <= 1e-8, "Legacy rotation round-trip failed.");
        return std::array<double,6>{{pose.position_m[0],pose.position_m[1],pose.position_m[2],roll,pitch,yaw}};
    });
}

Result<PrintJob> TransformToolpath(const PrintJob& job, const PlanningOptions& options) {
    Result<PrintJob> result;
    const auto imported = ValidateImportedJob(job);
    if(imported.status != ResultStatus::success) { result.status = imported.status; result.diagnostics = imported.diagnostics; return result; }
    if(!options.fixed_orientation || !options.fixed_rails || !options.model_identity.has_value) {
        result.status = ResultStatus::rejected;
        Diagnostic d; d.code = DiagnosticCode::missing_model_data;
        d.message = "Fixed orientation/rails and complete model identity are required for transformation."; result.diagnostics.push_back(d); return result;
    }
    try {
        PrintJob transformed = job;
        transformed.frames = options.frames; transformed.model_identity = options.model_identity;
        // Imported nozzle coordinates must retain their named source frame.
        Require(job.frames.bed_frame_id == options.frames.bed_frame_id
            && job.frames.nozzle_frame_id == options.frames.nozzle_frame_id, "Source and registered nozzle/bed frame names disagree.");
        for(auto& operation : transformed.operations) if(operation.motion.has_value) {
            auto& motion = operation.motion.value;
            const auto start = TransformNozzleToEndEffector(motion.nozzle.start,options.frames);
            const auto end = TransformNozzleToEndEffector(motion.nozzle.end,options.frames);
            if(start.status != ResultStatus::success || end.status != ResultStatus::success) {
                result.status = ResultStatus::rejected;
                result.diagnostics = start.status != ResultStatus::success ? start.diagnostics : end.diagnostics; return result;
            }
            const auto legacy = ToLegacyPose(start.value);
            if(legacy.status != ResultStatus::success) { result.status = legacy.status; result.diagnostics = legacy.diagnostics; return result; }
            motion.end_effector.has_value = true; motion.end_effector.value.frame_id = options.frames.world_frame_id;
            motion.end_effector.value.start = start.value; motion.end_effector.value.end = end.value;
        }
        transformed.stage = PlanningStage::transformed;
        Passed(transformed,ValidationCheckKind::coordinate_transform); Passed(transformed,ValidationCheckKind::orientation);
        const auto validated = ValidatePlanningArtifact(transformed);
        if(validated.status != ResultStatus::success) { result.status = validated.status; result.diagnostics = validated.diagnostics; return result; }
        transformed.job_id="content-v1";
        const auto encoded=SerializePrintJob(transformed);
        if(encoded.status!=ResultStatus::success) {result.status=encoded.status;result.diagnostics=encoded.diagnostics;return result;}
        const auto hash=ComputeSourceSha256(encoded.value);
        Require(hash.status==ResultStatus::success,"Transformed job identity hashing failed.");
        transformed.job_id="transform-"+hash.value;
        result.value = std::move(transformed); result.status = ResultStatus::success;
    } catch(const std::exception& error) {
        result.status = ResultStatus::rejected;
        Diagnostic d; d.code = DiagnosticCode::invalid_transform; d.message = error.what(); result.diagnostics.push_back(d);
    }
    return result;
}
} // namespace cdpr_slicer
