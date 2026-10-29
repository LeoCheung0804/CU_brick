#include "cdpr_slicer/Planner.h"
#include "cdpr_slicer/Mechanics.h"
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {
struct DomainError:std::runtime_error {
    DiagnosticCode code;
    DomainError(DiagnosticCode c,const std::string& text):std::runtime_error(text),code(c){}
};
void Require(bool condition,DiagnosticCode code,const std::string& text) {
    if(!condition) throw DomainError(code,text);
}
void Positive(double value,const std::string& name) {
    Require(std::isfinite(value)&&value>0,DiagnosticCode::invalid_geometry,name);
}
} // namespace

namespace {
Result<std::vector<double>> Checked(const RobotModelSnapshot& model,const std::array<double,6>& pose,
    std::array<CableFreeSpan,8>* spans) {
    Result<std::vector<double>> result;
    try {
        const auto& config=model.kinematics;
        Require(config.cableCount==8&&model.rail_count==4,DiagnosticCode::invalid_configuration,"S2-B requires eight cables and four rails.");
        for(auto value:pose) Require(std::isfinite(value),DiagnosticCode::invalid_geometry,"Nonfinite legacy pose.");
        Positive(config.pulleyRadius,"Pulley radius must be finite and positive.");
        for(std::size_t i=0;i<4;++i)
            Require(std::isfinite(model.rail_offsets_m[i])&&std::isfinite(config.startingRailOffsets[i]),DiagnosticCode::invalid_geometry,"Nonfinite rail state.");
        Eigen::Matrix3d rx,ry,rz;
        rx<<1,0,0,0,std::cos(pose[3]),-std::sin(pose[3]),0,std::sin(pose[3]),std::cos(pose[3]);
        ry<<std::cos(pose[4]),0,std::sin(pose[4]),0,1,0,-std::sin(pose[4]),0,std::cos(pose[4]);
        rz<<std::cos(pose[5]),-std::sin(pose[5]),0,std::sin(pose[5]),std::cos(pose[5]),0,0,0,1;
        const Eigen::Matrix3d rotation=rx*ry*rz;
        const Eigen::Vector3d position(pose[0],pose[1],pose[2]);
        // Domain guards mirror intermediate geometry only. Cable lengths are
        // obtained exclusively from the existing shared implementation below.
        for(std::size_t i=0;i<8;++i) {
            const auto label="Cable "+std::to_string(i)+": ";
            const auto& axis=config.pulleyAxes[i];
            Require(axis.allFinite()&&axis.x()==0&&axis.y()==0&&std::abs(axis.z())==1
                &&config.frameAttachments[i].allFinite()&&config.endEffectorAttachments[i].allFinite(),
                DiagnosticCode::invalid_geometry,label+"invalid legacy geometry/axis.");
            const Eigen::Vector3d outlet=rotation*config.endEffectorAttachments[i]+position;
            const Eigen::Vector3d normal=axis.cross(outlet-config.frameAttachments[i]);
            const Eigen::Vector3d radial=normal.cross(axis);
            Require(outlet.allFinite()&&normal.allFinite()&&radial.allFinite()
                &&std::isfinite(normal.norm())&&normal.norm()>1e-12&&std::isfinite(radial.norm())&&radial.norm()>1e-12,
                DiagnosticCode::invalid_geometry,label+"pulley plane/radial direction is degenerate.");
            Eigen::Vector3d center=radial/radial.norm()*config.pulleyRadius+config.frameAttachments[i];
            if(i<4) center.z()+=model.rail_offsets_m[i];
            const double distance=(outlet-center).norm();
            Require(center.allFinite()&&std::isfinite(distance)&&distance>config.pulleyRadius,
                DiagnosticCode::invalid_geometry,label+"no real nondegenerate pulley tangent.");
            const double ratio=config.pulleyRadius/distance;
            Require(std::isfinite(ratio)&&ratio>0&&ratio<1,DiagnosticCode::invalid_geometry,label+"tangent ratio outside (0,1).");
            const Eigen::Vector3d pulley_outlet=center+ratio*ratio*(outlet-center)
                -ratio*std::sqrt(1-ratio*ratio)*(normal/normal.norm()).cross(outlet-center);
            const double outlet_norm=(pulley_outlet-center).norm();
            Require(pulley_outlet.allFinite()&&std::isfinite(outlet_norm)&&outlet_norm>0,
                DiagnosticCode::invalid_geometry,label+"invalid pulley outlet.");
            if(spans) for(std::size_t axis_index=0;axis_index<3;++axis_index) {
                (*spans)[i].pulley_m[axis_index]=pulley_outlet[static_cast<int>(axis_index)];
                (*spans)[i].attachment_m[axis_index]=outlet[static_cast<int>(axis_index)];
            }
            const double cosine=(-radial/radial.norm()).dot((pulley_outlet-center)/outlet_norm);
            Require(std::isfinite(cosine)&&cosine>=-1&&cosine<=1,DiagnosticCode::invalid_geometry,label+"acos domain failure.");
        }
        auto lengths=cu_brick::ComputeCableLengths(config,pose,model.rail_offsets_m);
        Require(lengths.size()==8,DiagnosticCode::invalid_geometry,"Shared kinematics returned the wrong cable count.");
        for(std::size_t i=0;i<8;++i) {
            const auto label="Cable "+std::to_string(i)+": "; Positive(lengths[i],label+"nonpositive/nonfinite shared cable length.");
            if(model.limits.minimum_cable_length_m.has_value) {
                const double lower=model.limits.minimum_cable_length_m.value[i];
                Require(std::isfinite(lower)&&lower>=0,DiagnosticCode::invalid_configuration,label+"invalid lower length limit.");
                Require(lengths[i]>=lower,DiagnosticCode::outside_workspace,label+"below configured cable length.");
            }
            if(model.limits.maximum_cable_length_m.has_value) {
                const double upper=model.limits.maximum_cable_length_m.value[i]; Positive(upper,label+"invalid upper length limit.");
                Require(lengths[i]<=upper,DiagnosticCode::outside_workspace,label+"above configured cable length.");
            }
            const double scale=model.cable_scale_counts_per_m[i];
            Require(std::isfinite(scale)&&scale!=0,DiagnosticCode::invalid_configuration,label+"unknown cable-to-count scale.");
            const double count=lengths[i]*scale;
            Require(std::isfinite(count)&&count>=static_cast<double>(std::numeric_limits<std::int32_t>::min())
                &&count<=static_cast<double>(std::numeric_limits<std::int32_t>::max()),DiagnosticCode::limit_exceeded,label+"int32 motor count overflow.");
            const auto command=static_cast<std::int32_t>(count); // Match Robot's truncation, after range checks.
            if(model.limits.minimum_motor_count.has_value)
                Require(command>=model.limits.minimum_motor_count.value[i],DiagnosticCode::outside_workspace,label+"below configured count bound.");
            if(model.limits.maximum_motor_count.has_value)
                Require(command<=model.limits.maximum_motor_count.value[i],DiagnosticCode::outside_workspace,label+"above configured count bound.");
        }
        result.value=std::move(lengths); result.status=ResultStatus::success;
    } catch(const DomainError& error) {
        result.status=ResultStatus::rejected; Diagnostic d; d.code=error.code; d.message=error.what(); result.diagnostics.push_back(d);
    } catch(const std::exception&) {
        result.status=ResultStatus::fault; Diagnostic d; d.code=DiagnosticCode::execution_fault; d.message="Checked shared kinematics could not complete."; result.diagnostics.push_back(d);
    }
    return result;
}
} // namespace
Result<std::vector<double>> ComputeCheckedCableLengths(const RobotModelSnapshot& model,const std::array<double,6>& pose) {
    return Checked(model,pose,nullptr);
}
Result<std::array<CableFreeSpan,8>> ComputeCheckedCableSpans(const RobotModelSnapshot& model,const std::array<double,6>& pose) {
    Result<std::array<CableFreeSpan,8>> result;
    const auto checked=Checked(model,pose,&result.value);
    result.status=checked.status; result.diagnostics=checked.diagnostics;
    if(result.status!=ResultStatus::success) result.value={};
    return result;
}
} // namespace cdpr_slicer
