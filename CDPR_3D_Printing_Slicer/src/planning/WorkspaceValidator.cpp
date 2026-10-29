#include "cdpr_slicer/Planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {
void Error(Result<ValidationReport>& result,DiagnosticCode code,const char* message,ResultStatus status=ResultStatus::rejected) {
    result.status=status; Diagnostic d; d.code=code; d.message=message; result.diagnostics.push_back(d);
}
bool Identity(const ModelIdentity& a,const ModelIdentity& b) {
    return a.robot_config_sha256==b.robot_config_sha256&&a.calibration_sha256==b.calibration_sha256
        &&a.rail_state_sha256==b.rail_state_sha256&&a.motion_limits_sha256==b.motion_limits_sha256;
}
void Status(ValidationReport& report,ValidationCheckKind kind,ValidationStatus status,const std::string& message) {
    for(auto& check:report.checks) if(check.kind==kind) {
        check.status=status;
        if(!message.empty()) {Diagnostic d; d.severity=Severity::info; d.code=DiagnosticCode::metadata; d.message=message; check.diagnostics.push_back(d);}
    }
}
} // namespace
Result<ValidationReport> ValidateWorkspace(const PrintJob& job,const RobotModelSnapshot& model,const PlanningOptions& options) {
    Result<ValidationReport> result;
    try {
        const auto artifact=ValidatePlanningArtifact(job);
        if(artifact.status!=ResultStatus::success) {result.status=artifact.status; result.diagnostics=artifact.diagnostics; return result;}
        if(job.stage!=PlanningStage::transformed) {Error(result,DiagnosticCode::invalid_geometry,"Workspace validation requires a transformed, untimed job."); return result;}
        const auto bound=BindPlanningOptions(options,model);
        if(bound.status!=ResultStatus::success) {result.status=bound.status; result.diagnostics=bound.diagnostics; return result;}
        const auto& plan=bound.value;
        if(!job.model_identity.has_value||!Identity(job.model_identity.value,model.identity)) {
            Error(result,DiagnosticCode::identity_mismatch,"Toolpath and model identities disagree."); return result;
        }
        const auto job_frames=SerializeFrameRegistration(job.frames), model_frames=SerializeFrameRegistration(model.frames);
        if(job_frames.status!=ResultStatus::success||model_frames.status!=ResultStatus::success||job_frames.value!=model_frames.value) {
            Error(result,DiagnosticCode::identity_mismatch,"Toolpath registration disagrees with the bound calibration."); return result;
        }
        if(!plan.path_tolerance_m.has_value||!std::isfinite(plan.path_tolerance_m.value)||plan.path_tolerance_m.value<=0||!plan.limits.workspace.has_value) {
            Error(result,DiagnosticCode::missing_model_data,"Explicit spatial sampling interval and XYZ envelope are required.",ResultStatus::indeterminate); return result;
        }
        std::size_t diagnostics=job.validation.diagnostics.size();
        for(const auto& check:job.validation.checks) diagnostics+=check.diagnostics.size();
        if(diagnostics>9990) {Error(result,DiagnosticCode::limit_exceeded,"Reserve diagnostic capacity for geometric/planning checks."); return result;}
        std::size_t samples=0;
        for(const auto& operation:job.operations) if(operation.motion.has_value) {
            const auto& motion=operation.motion.value; const auto& geometry=motion.end_effector.value;
            double length=0;
            for(std::size_t axis=0;axis<3;++axis) length=std::hypot(length,geometry.end.position_m[axis]-geometry.start.position_m[axis]);
            const double required=std::max(1.0,std::ceil(length/plan.path_tolerance_m.value));
            if(!std::isfinite(length)||!std::isfinite(required)||required>plan.maximum_samples_per_motion-1) {
                Error(result,DiagnosticCode::limit_exceeded,"Geometric subdivision exceeds per-motion sample capacity."); return result;
            }
            const auto steps=static_cast<std::size_t>(required);
            if(steps+1>plan.maximum_samples-samples) {Error(result,DiagnosticCode::limit_exceeded,"Geometric sampling exceeds job capacity."); return result;}
            samples+=steps+1;
            for(std::size_t i=0;i<=steps;++i) {
                const double fraction=static_cast<double>(i)/static_cast<double>(steps); auto pose=geometry.start;
                for(std::size_t axis=0;axis<3;++axis) {
                    pose.position_m[axis]=i==0?geometry.start.position_m[axis]:i==steps?geometry.end.position_m[axis]
                        :(1-fraction)*geometry.start.position_m[axis]+fraction*geometry.end.position_m[axis];
                    if(!std::isfinite(pose.position_m[axis])||pose.position_m[axis]<plan.limits.workspace.value.minimum_m[axis]
                        ||pose.position_m[axis]>plan.limits.workspace.value.maximum_m[axis]) {
                        Error(result,DiagnosticCode::outside_workspace,"Transformed EE path exceeds the configured XYZ envelope.");
                        result.diagnostics.back().operation_id.has_value=true; result.diagnostics.back().operation_id.value=operation.id;
                        result.diagnostics.back().source.has_value=true; result.diagnostics.back().source.value=operation.source; return result;
                    }
                }
                const auto legacy=ToLegacyPose(pose);
                if(legacy.status!=ResultStatus::success) {result.status=legacy.status; result.diagnostics=legacy.diagnostics; return result;}
                const auto cables=ComputeCheckedCableLengths(model,legacy.value);
                if(cables.status!=ResultStatus::success) {
                    result.status=cables.status; result.diagnostics=cables.diagnostics;
                    for(auto& diagnostic:result.diagnostics) {
                        diagnostic.operation_id.has_value=true; diagnostic.operation_id.value=operation.id;
                        diagnostic.source.has_value=true; diagnostic.source.value=operation.source;
                    } return result;
                }
            }
        }
        auto report=job.validation; report.hardware_ready=false;
        Status(report,ValidationCheckKind::geometry,ValidationStatus::passed,
            "Shared cable geometry/domain and length/count checks passed at "+std::to_string(samples)+" spatial samples; this is not a continuous feasibility proof.");
        Status(report,ValidationCheckKind::workspace,ValidationStatus::passed,
            "EE segment endpoints/interior are inside the configured XYZ envelope; collision, cable tension and robot singularity are unevaluated.");
        const bool counts=plan.limits.minimum_motor_count.has_value&&plan.limits.maximum_motor_count.has_value;
        Status(report,ValidationCheckKind::motor_count,counts?ValidationStatus::passed:ValidationStatus::not_evaluated,
            counts?"Configured count bounds passed at spatial samples.":"Only int32 representability is known; complete configured motor bounds are missing.");
        result.value=std::move(report); result.status=ResultStatus::success;
    } catch(const std::exception&) {Error(result,DiagnosticCode::execution_fault,"Offline workspace validation could not complete.",ResultStatus::fault);}
    return result;
}
Result<PrintJob> CreateGeometricPlan(const PrintJob& job,const RobotModelSnapshot& model,const PlanningOptions& options) {
    Result<PrintJob> result;
    try {
        const auto binding=BindPlanningOptions(options,model);
        if(binding.status!=ResultStatus::success) {result.status=binding.status;result.diagnostics=binding.diagnostics;return result;}
        PrintJob checked=job;
        const auto artifact=ValidatePlanningArtifact(job);
        if(artifact.status!=ResultStatus::success) {result.status=artifact.status;result.diagnostics=artifact.diagnostics;return result;}
        if(job.stage==PlanningStage::imported) {
            const auto transformed=TransformToolpath(job,binding.value);
            if(transformed.status!=ResultStatus::success) {result.status=transformed.status;result.diagnostics=transformed.diagnostics;return result;}
            checked=transformed.value;
        } else if(job.stage==PlanningStage::geometrically_checked) {
            // Recompute against the selected snapshot instead of trusting stored
            // geometric claims when an intermediate job is opened again.
            checked.stage=PlanningStage::transformed;
            for(auto& check:checked.validation.checks) if(check.kind==ValidationCheckKind::geometry
                ||check.kind==ValidationCheckKind::workspace||check.kind==ValidationCheckKind::motor_count) {
                check.status=ValidationStatus::not_evaluated;check.diagnostics.clear();
            }
        }
        const auto report=ValidateWorkspace(checked,model,binding.value);
        if(report.status!=ResultStatus::success) {result.status=report.status;result.diagnostics=report.diagnostics;return result;}
        checked.validation=report.value;checked.stage=PlanningStage::geometrically_checked;
        checked.job_id="content-v1";
        const auto encoded=SerializePrintJob(checked);
        if(encoded.status!=ResultStatus::success) {result.status=encoded.status;result.diagnostics=encoded.diagnostics;return result;}
        const auto hash=ComputeSourceSha256(encoded.value);
        if(hash.status!=ResultStatus::success) {result.status=hash.status;result.diagnostics=hash.diagnostics;return result;}
        checked.job_id="geometry-"+hash.value;result.value=std::move(checked);result.status=ResultStatus::success;
    } catch(const std::exception&) {
        result.status=ResultStatus::fault;Diagnostic d;d.code=DiagnosticCode::execution_fault;d.message="Geometric job creation could not complete.";result.diagnostics.push_back(d);
    }
    return result;
}
} // namespace cdpr_slicer
