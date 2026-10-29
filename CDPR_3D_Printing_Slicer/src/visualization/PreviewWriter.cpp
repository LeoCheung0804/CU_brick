#include "cdpr_slicer/Planner.h"
#include "cdpr_slicer/Mechanics.h"
#include "PreviewTemplate.h"
#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {
using Json=nlohmann::json;
struct PreviewError:std::runtime_error {
    DiagnosticCode code;
    PreviewError(DiagnosticCode c,const std::string& message):std::runtime_error(message),code(c){}
};
std::string InertJson(const Json& value) {
    const auto ascii=value.dump(-1,' ',true);
    std::string result; result.reserve(ascii.size());
    for(char c:ascii) {
        if(c=='<') result+="\\u003c";
        else if(c=='>') result+="\\u003e";
        else if(c=='&') result+="\\u0026";
        else result+=c;
    }
    return result;
}
Json Vector(const Eigen::Vector3d& value) {
    return Json::array({value.x(),value.y(),value.z()});
}
Json RobotData(const PrintJob& job,const RobotModelSnapshot& model,std::size_t motion_count) {
    const auto valid=ValidateModelSnapshot(model);
    if(valid.status!=ResultStatus::success||!model.has_calibration||!model.has_motion_limits||!job.model_identity.has_value)
        throw PreviewError(DiagnosticCode::identity_mismatch,"3D robot preview requires a verified model bound to the planning artifact.");
    const auto& a=job.model_identity.value; const auto& b=model.identity;
    if(a.robot_config_sha256!=b.robot_config_sha256||a.calibration_sha256!=b.calibration_sha256
        ||a.rail_state_sha256!=b.rail_state_sha256||a.motion_limits_sha256!=b.motion_limits_sha256)
        throw PreviewError(DiagnosticCode::identity_mismatch,"3D preview model differs from the artifact's recorded model identity.");
    const auto job_frames=SerializeFrameRegistration(job.frames),model_frames=SerializeFrameRegistration(model.frames);
    if(job_frames.status!=ResultStatus::success||model_frames.status!=ResultStatus::success||job_frames.value!=model_frames.value)
        throw PreviewError(DiagnosticCode::identity_mismatch,"3D preview registration differs from the bound model.");
    Json robot={{"frame_anchors_m",Json::array()},{"ee_attachments_m",Json::array()},
        {"rail_offsets_m",model.rail_offsets_m},{"pulley_radius_m",model.kinematics.pulleyRadius},
        {"model_sha256",b.robot_config_sha256},{"workspace",nullptr},{"cable_samples",Json::array()}};
    if(model.limits.workspace.has_value) robot["workspace"]={{"minimum_m",model.limits.workspace.value.minimum_m},
        {"maximum_m",model.limits.workspace.value.maximum_m}};
    for(std::size_t i=0;i<model.kinematics.cableCount;++i) {
        Eigen::Vector3d anchor=model.kinematics.frameAttachments[i];
        if(i<cu_brick::kMaximumRailCount) anchor.z()+=model.rail_offsets_m[i];
        robot["frame_anchors_m"].push_back(Vector(anchor));
        robot["ee_attachments_m"].push_back(Vector(model.kinematics.endEffectorAttachments[i]));
    }
    // Bound display samples to 40,000. Lengths use the original C++ solver;
    // the browser only interpolates those samples and labels them approximate.
    const std::size_t points=std::min<std::size_t>(33,40000/std::max<std::size_t>(1,motion_count));
    for(const auto& operation:job.operations) if(operation.motion.has_value) {
        const auto& motion=operation.motion.value;
        if(!motion.end_effector.has_value)
            throw PreviewError(DiagnosticCode::invalid_geometry,"Robot playback requires derived end-effector geometry.");
        const auto& geometry=motion.end_effector.value;
        Json samples=Json::array();
        for(std::size_t i=0;i<points;++i) {
            const double u=static_cast<double>(i)/static_cast<double>(points-1);
            const double progress=operation.timing.has_value?u*u*u*(10+u*(-15+6*u)):u;
            auto pose=geometry.start;
            for(std::size_t axis=0;axis<3;++axis) pose.position_m[axis]=i==0?geometry.start.position_m[axis]
                :i+1==points?geometry.end.position_m[axis]
                :geometry.start.position_m[axis]+(geometry.end.position_m[axis]-geometry.start.position_m[axis])*progress;
            const auto legacy=ToLegacyPose(pose);
            if(legacy.status!=ResultStatus::success)
                throw PreviewError(DiagnosticCode::invalid_geometry,"Preview sample cannot use the legacy orientation convention.");
            const auto lengths=ComputeCheckedCableLengths(model,legacy.value);
            if(lengths.status!=ResultStatus::success)
                throw PreviewError(DiagnosticCode::invalid_geometry,"A display sample failed the shared cable-geometry checks; no robot preview was exported.");
            samples.push_back({{"fraction",u},{"lengths_m",lengths.value}});
        }
        robot["cable_samples"].push_back({{"operation_id",operation.id},{"samples",std::move(samples)}});
    }
    return robot;
}
Result<std::string> Preview(const PrintJob& job,const RobotModelSnapshot* model,const std::string* assessment=nullptr) {
    Result<std::string> result;
    const auto encoded=SerializePrintJob(job);
    if(encoded.status!=ResultStatus::success) {result.status=encoded.status; result.diagnostics=encoded.diagnostics; return result;}
    try {
        std::size_t motions=0;
        for(const auto& operation:job.operations) if(operation.motion.has_value) ++motions;
        if(motions>20000) throw PreviewError(DiagnosticCode::limit_exceeded,"Preview supports at most 20,000 original motion operations.");
        for(bool ee:{false,true}) for(std::size_t axis=0;axis<3;++axis) {
            double lower=std::numeric_limits<double>::infinity(),upper=-lower;
            for(const auto& operation:job.operations) if(operation.motion.has_value) {
                const auto& motion=operation.motion.value;
                if(ee&&!motion.end_effector.has_value) continue;
                const auto& g=ee?motion.end_effector.value:motion.nozzle;
                lower=std::min(lower,std::min(g.start.position_m[axis],g.end.position_m[axis]));
                upper=std::max(upper,std::max(g.start.position_m[axis],g.end.position_m[axis]));
            }
            if(std::isfinite(lower)&&!std::isfinite(upper-lower))
                throw PreviewError(DiagnosticCode::limit_exceeded,"Preview coordinate range exceeds the renderer's numeric envelope.");
        }
        auto robot=model?RobotData(job,*model,motions):Json(nullptr);
        if(assessment) {
            if(!model) throw PreviewError(DiagnosticCode::missing_model_data,"Mechanics preview requires the selected model.");
            const auto valid=ValidateMechanicsAssessment(job,*model,*assessment);
            if(valid.status!=ResultStatus::success) throw PreviewError(DiagnosticCode::identity_mismatch,"Mechanics report is stale, changed, or bound to a different job/model.");
            robot["mechanics"]=Json::parse(*assessment);
        }
        auto html=std::string(kPreviewTemplate);
        const std::string job_token="__CDPR_JOB_DATA__",robot_token="__CDPR_ROBOT_DATA__";
        const auto job_offset=html.find(job_token),robot_offset=html.find(robot_token);
        if(job_offset==std::string::npos||robot_offset==std::string::npos||robot_offset<=job_offset)
            throw std::runtime_error("Invalid bundled preview template.");
        // Replace original offsets in reverse order: marker text inside user
        // data must never redirect a subsequent template substitution.
        html.replace(robot_offset,robot_token.size(),InertJson(robot));
        html.replace(job_offset,job_token.size(),InertJson(Json::parse(encoded.value)));
        if(html.size()>64u*1024u*1024u) throw PreviewError(DiagnosticCode::limit_exceeded,"Preview byte limit exceeded.");
        result.value=std::move(html); result.status=ResultStatus::success;
    } catch(const PreviewError& error) {
        result.status=ResultStatus::rejected; Diagnostic d; d.code=error.code; d.message=error.what(); result.diagnostics.push_back(d);
    } catch(const std::exception&) {
        result.status=ResultStatus::fault; Diagnostic d; d.code=DiagnosticCode::execution_fault;
        d.message="Offline preview generation could not complete."; result.diagnostics.push_back(d);
    }
    return result;
}
} // namespace
Result<std::string> CreatePreviewHtml(const PrintJob& job) {return Preview(job,nullptr);}
Result<std::string> CreatePreviewHtml(const PrintJob& job,const RobotModelSnapshot& model) {return Preview(job,&model);}
Result<std::string> CreatePreviewHtml(const PrintJob& job,const RobotModelSnapshot& model,const std::string& assessment) {return Preview(job,&model,&assessment);}
} // namespace cdpr_slicer
