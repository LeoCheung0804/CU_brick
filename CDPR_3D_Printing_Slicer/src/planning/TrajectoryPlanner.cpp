#include "cdpr_slicer/Planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {
struct PlanningError:std::runtime_error {
    DiagnosticCode code; ResultStatus status;
    PlanningError(DiagnosticCode c,const char* text,ResultStatus s=ResultStatus::rejected):std::runtime_error(text),code(c),status(s){}
};
void Require(bool ok,const char* message,DiagnosticCode code=DiagnosticCode::invalid_timing) {
    if(!ok) throw PlanningError(code,message);
}
double Known(const OptionalValue<double>& value,const char* name) {
    if(!value.has_value) throw PlanningError(DiagnosticCode::missing_model_data,name,ResultStatus::indeterminate);
    Require(std::isfinite(value.value)&&value.value>0,name,DiagnosticCode::invalid_configuration); return value.value;
}
double Length(const MotionGeometry& geometry) {
    double length=0;
    for(std::size_t i=0;i<3;++i) length=std::hypot(length,geometry.end.position_m[i]-geometry.start.position_m[i]);
    Require(std::isfinite(length),"Path-length arithmetic overflow.",DiagnosticCode::invalid_geometry); return length;
}
void Note(ValidationReport& report,ValidationCheckKind kind,ValidationStatus status,const std::string& text) {
    for(auto& check:report.checks) if(check.kind==kind) {
        check.status=status;
        if(!text.empty()) {Diagnostic d; d.severity=Severity::info; d.code=DiagnosticCode::metadata; d.message=text; check.diagnostics.push_back(d);}
    }
}
struct SampledMotion {
    std::vector<ProfileKnot> profile;
    std::vector<std::array<double,8>> cable_lengths;
};
SampledMotion Sample(const MotionOperation& motion,double duration,const RobotModelSnapshot& model,
    const PlanningOptions& options,std::size_t remaining) {
    const double length=Length(motion.nozzle);
    const double intervals=std::max(4.0,std::max(std::ceil(duration/options.sample_period_s.value),
        std::ceil(1.875*length/options.path_tolerance_m.value)));
    Require(std::isfinite(intervals)&&intervals<=options.maximum_samples_per_motion-1,"Temporal/spatial sample capacity exceeded.",DiagnosticCode::limit_exceeded);
    const auto n=static_cast<std::size_t>(intervals);
    Require(n+1<=remaining,"Job profile sample capacity exceeded.",DiagnosticCode::limit_exceeded);
    SampledMotion output; output.profile.reserve(n+1); output.cable_lengths.reserve(n+1); double previous=0;
    for(std::size_t i=0;i<=n;++i) {
        ProfileKnot knot;
        knot.offset_s=i==n?duration:duration*static_cast<double>(i)/static_cast<double>(n);
        const double u=knot.offset_s/duration;
        const double s=i==n?1:std::max(previous,std::max(0.0,std::min(1.0,u*u*u*(10+u*(-15+6*u)))));
        const double rate=(i==0||i==n)?0:30*u*u*(1-u)*(1-u)/duration;
        knot.progress=s; knot.filament_offset_mm=i==n?motion.delta_filament_mm:motion.delta_filament_mm*s;
        knot.filament_rate_mm_per_s=motion.delta_filament_mm*rate;
        if(length>0) {knot.path_speed_m_per_s.has_value=true; knot.path_speed_m_per_s.value=length*rate;}
        Require(std::isfinite(knot.offset_s)&&std::isfinite(knot.filament_offset_mm)&&std::isfinite(knot.filament_rate_mm_per_s)
            &&(i==0||knot.offset_s>output.profile.back().offset_s),"Profile precision/range failure.");
        auto pose=motion.end_effector.value.start;
        for(std::size_t axis=0;axis<3;++axis) pose.position_m[axis]=i==0?motion.end_effector.value.start.position_m[axis]
            :i==n?motion.end_effector.value.end.position_m[axis]
            :(1-s)*motion.end_effector.value.start.position_m[axis]+s*motion.end_effector.value.end.position_m[axis];
        const auto legacy=ToLegacyPose(pose);
        Require(legacy.status==ResultStatus::success,"Profile orientation cannot be converted.",DiagnosticCode::invalid_transform);
        const auto cables=ComputeCheckedCableLengths(model,legacy.value);
        if(cables.status!=ResultStatus::success)
            throw PlanningError(cables.diagnostics.empty()?DiagnosticCode::invalid_geometry:cables.diagnostics.front().code,
                "Cable geometry/length/count validation failed during temporal sampling.");
        std::array<double,8> values{}; std::copy(cables.value.begin(),cables.value.end(),values.begin());
        output.profile.push_back(knot); output.cable_lengths.push_back(values); previous=s;
    }
    return output;
}
std::pair<double,double> Envelope(const SampledMotion& samples,const MotionLimits& limits) {
    double speed_ratio=0,acceleration_ratio=0;
    for(std::size_t cable=0;cable<8;++cable) {
        const double vmax=limits.maximum_cable_speed_m_per_s.value[cable];
        const double amax=limits.maximum_cable_acceleration_m_per_s2.value[cable];
        Require(std::isfinite(vmax)&&vmax>0&&std::isfinite(amax)&&amax>0,"Cable dynamic limits must be finite and positive.",DiagnosticCode::invalid_configuration);
        double previous_velocity=0,previous_dt=0,last_velocity=0,last_dt=0;
        for(std::size_t i=1;i<samples.profile.size();++i) {
            const double dt=samples.profile[i].offset_s-samples.profile[i-1].offset_s;
            const double velocity=(samples.cable_lengths[i][cable]-samples.cable_lengths[i-1][cable])/dt;
            const double acceleration=i==1?2*velocity/dt:2*(velocity-previous_velocity)/(dt+previous_dt);
            Require(std::isfinite(velocity)&&std::isfinite(acceleration),"Cable finite-difference overflow.");
            speed_ratio=std::max(speed_ratio,std::abs(velocity)/vmax);
            acceleration_ratio=std::max(acceleration_ratio,std::abs(acceleration)/amax);
            previous_velocity=last_velocity=velocity; previous_dt=last_dt=dt;
        }
        acceleration_ratio=std::max(acceleration_ratio,std::abs(2*last_velocity/last_dt)/amax);
    }
    Require(std::isfinite(speed_ratio)&&std::isfinite(acceleration_ratio),"Cable envelope overflow.");
    return {speed_ratio,acceleration_ratio};
}
double InitialDuration(const MotionOperation& motion,const PlanningOptions& options) {
    const double length=Length(motion.nozzle), filament=std::abs(motion.delta_filament_mm);
    double duration=4*options.sample_period_s.value;
    if(length>0) {
        const double speed=std::min(motion.path_feedrate_m_per_s.value,Known(options.limits.maximum_path_speed_m_per_s,"A path speed limit is required."));
        const double acceleration=Known(options.limits.maximum_path_acceleration_m_per_s2,"A path acceleration limit is required.");
        duration=std::max(duration,1.875*length/speed);
        duration=std::max(duration,std::sqrt((10*std::sqrt(3.0)/3)*length/acceleration));
    }
    if(filament>0) {
        double rate=Known(options.limits.maximum_filament_rate_mm_per_s,"A filament rate limit is required.");
        if(length==0) rate=std::min(rate,motion.filament_feedrate_mm_per_s.value);
        duration=std::max(duration,1.875*filament/rate);
        if(motion.delta_filament_mm>0&&options.limits.maximum_volumetric_flow_mm3_per_s.has_value) {
            const double diameter=Known(options.filament_diameter_mm,"Filament diameter is required for a configured volume-flow limit.");
            const double area=std::acos(-1.0)*diameter*diameter/4;
            Require(std::isfinite(area)&&area>0,"Invalid filament area.",DiagnosticCode::invalid_configuration);
            const double volumetric=Known(options.limits.maximum_volumetric_flow_mm3_per_s,"Invalid volume-flow limit.");
            duration=std::max(duration,1.875*filament*area/volumetric);
        }
    }
    Require(std::isfinite(duration)&&duration>0&&duration<=86400,"Motion duration exceeds the planning envelope.",DiagnosticCode::limit_exceeded);
    return duration;
}
} // namespace

Result<PrintJob> PlanTrajectory(const PrintJob& job,const RobotModelSnapshot& model,const PlanningOptions& options) {
    Result<PrintJob> result; OptionalValue<std::uint64_t> current;
    try {
        Require(job.stage==PlanningStage::imported||job.stage==PlanningStage::transformed||job.stage==PlanningStage::geometrically_checked,
            "Plan from an imported, transformed or geometrically checked untimed job.",DiagnosticCode::invalid_geometry);
        const auto binding=BindPlanningOptions(options,model);
        if(binding.status!=ResultStatus::success) {result.status=binding.status; result.diagnostics=binding.diagnostics; return result;}
        const auto& bound=binding.value;
        Known(bound.sample_period_s,"A positive explicit sampling period is required.");
        Known(bound.path_tolerance_m,"A positive explicit spatial sampling interval is required.");
        const auto geometry=CreateGeometricPlan(job,model,bound);
        if(geometry.status!=ResultStatus::success) {result.status=geometry.status; result.diagnostics=geometry.diagnostics; return result;}
        PrintJob planned=geometry.value;
        std::uint64_t epoch=0; double cursor=0; std::size_t samples=0;
        for(auto& operation:planned.operations) {
            current.has_value=true; current.value=operation.id;
            OperationTiming timing; timing.epoch_id=epoch; timing.start_s=cursor; timing.interpolation="event_v1";
            if(operation.motion.has_value) {
                if(!bound.limits.maximum_cable_speed_m_per_s.has_value||!bound.limits.maximum_cable_acceleration_m_per_s2.has_value)
                    throw PlanningError(DiagnosticCode::missing_model_data,"All eight cable speed/acceleration limits are required.",ResultStatus::indeterminate);
                double duration=InitialDuration(operation.motion.value,bound); bool accepted=false;
                for(unsigned attempt=0;attempt<12;++attempt) {
                    auto sampled=Sample(operation.motion.value,duration,model,bound,bound.maximum_samples-samples);
                    const auto envelope=Envelope(sampled,bound.limits);
                    if(envelope.first<=1&&envelope.second<=1) {
                        samples+=sampled.profile.size(); timing.profile=std::move(sampled.profile);
                        timing.duration_s=duration; timing.interpolation="quintic_progress_v1"; accepted=true; break;
                    }
                    duration*=1.05*std::max(envelope.first,std::sqrt(envelope.second));
                    Require(std::isfinite(duration)&&duration<=86400,"Cable-limited retiming exceeds the duration envelope.",DiagnosticCode::limit_exceeded);
                }
                Require(accepted,"Cable finite-difference retiming did not converge within 12 attempts.",DiagnosticCode::limit_exceeded);
            } else if(operation.dwell.has_value) timing.duration_s=operation.dwell.value.duration_s;
            else if(operation.kind==OperationKind::wait_temperature) {
                auto& temperature=operation.temperature.value;
                const auto& policy=temperature.heater==HeaterKind::hotend?bound.temperature_wait_policy.hotend:bound.temperature_wait_policy.bed;
                Known(policy.tolerance_deg_c,"Temperature waits require an explicit positive tolerance.");
                Require(Known(policy.timeout_s,"Temperature waits require an explicit bounded timeout.")<=86400,"Thermal wait timeout exceeds one day.",DiagnosticCode::invalid_configuration);
                temperature.tolerance_deg_c=policy.tolerance_deg_c; temperature.timeout_s=policy.timeout_s;
            }
            Require(std::isfinite(timing.duration_s)&&timing.duration_s>=0&&timing.duration_s<=86400,"Invalid event duration.");
            cursor+=timing.duration_s;
            Require(std::isfinite(cursor)&&cursor<=604800,"Epoch duration exceeds seven days.",DiagnosticCode::limit_exceeded);
            operation.timing.has_value=true; operation.timing.value=std::move(timing);
            if(operation.kind==OperationKind::wait_temperature) {++epoch; cursor=0;}
        }
        planned.stage=PlanningStage::timed;
        Note(planned.validation,ValidationCheckKind::execution_timing,ValidationStatus::passed,
            "Quintic progress per original segment; every segment begins/ends with zero velocity and acceleration. Waits create new relative epochs.");
        Note(planned.validation,ValidationCheckKind::cable_speed,ValidationStatus::not_evaluated,
            "Finite-difference speed envelope met supplied caps at "+std::to_string(samples)+" temporal knots; continuous extrema are not certified.");
        Note(planned.validation,ValidationCheckKind::cable_acceleration,ValidationStatus::not_evaluated,
            "Finite-difference acceleration envelope met supplied caps, including segment start/stop; continuous extrema are not certified.");
        const auto valid=ValidatePlanningArtifact(planned);
        if(valid.status!=ResultStatus::success) {result.status=valid.status; result.diagnostics=valid.diagnostics; return result;}
        planned.job_id="content-v1";
        const auto encoded=SerializePrintJob(planned);
        if(encoded.status!=ResultStatus::success) {result.status=encoded.status; result.diagnostics=encoded.diagnostics; return result;}
        const auto hash=ComputeSourceSha256(encoded.value);
        Require(hash.status==ResultStatus::success,"Plan identity hashing failed.",DiagnosticCode::execution_fault);
        planned.job_id="plan-"+hash.value;
        result.value=std::move(planned); result.status=ResultStatus::success;
    } catch(const PlanningError& error) {
        result.status=error.status; Diagnostic d; d.code=error.code; d.message=error.what(); d.operation_id=current; result.diagnostics.push_back(d);
    } catch(const std::exception&) {
        result.status=ResultStatus::fault; Diagnostic d; d.code=DiagnosticCode::execution_fault; d.message="Offline trajectory planning could not complete."; d.operation_id=current; result.diagnostics.push_back(d);
    }
    return result;
}
} // namespace cdpr_slicer
