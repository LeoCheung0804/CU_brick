#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#include <windows.h>
#endif
#include "cdpr_slicer/Planner.h"
#include "json.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace cdpr_slicer {
namespace {
using Json=nlohmann::json;
struct ModelError:std::runtime_error {
    DiagnosticCode code;
    ModelError(DiagnosticCode c,const std::string& text):std::runtime_error(text),code(c){}
};
void Require(bool ok,const char* text,DiagnosticCode code=DiagnosticCode::invalid_configuration) {
    if(!ok) throw ModelError(code,text);
}
template<typename T,typename F> Result<T> Attempt(F action) {
    Result<T> result;
    try {result.value=action(); result.status=ResultStatus::success;}
    catch(const ModelError& error) {result.status=ResultStatus::rejected; Diagnostic d; d.code=error.code; d.message=error.what(); result.diagnostics.push_back(d);}
    catch(const Json::exception&) {result.status=ResultStatus::rejected; Diagnostic d; d.code=DiagnosticCode::malformed_input; d.message="Malformed model/manifest JSON."; result.diagnostics.push_back(d);}
    catch(const std::exception&) {result.status=ResultStatus::fault; Diagnostic d; d.code=DiagnosticCode::execution_fault; d.message="Offline model processing could not complete."; result.diagnostics.push_back(d);}
    return result;
}
std::string ReadFileBytes(const std::string& path) {
#ifdef _WIN32
    Require(!path.empty()&&path.size()<960,"Model paths must be explicit local absolute paths.");
    const int n=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.data(),static_cast<int>(path.size()),nullptr,0);
    Require(n>0,"Model path is not UTF-8."); std::wstring wide(static_cast<std::size_t>(n),L'\0');
    Require(MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path.data(),static_cast<int>(path.size()),&wide[0],n)==n,"Path conversion failed.");
    std::replace(wide.begin(),wide.end(),L'/',L'\\');
    Require(wide.size()>=3&&wide.size()<240&&((wide[0]>=L'A'&&wide[0]<=L'Z')||(wide[0]>=L'a'&&wide[0]<=L'z'))
        &&wide[1]==L':'&&wide[2]==L'\\',"Model paths must be local drive paths.");
    const auto drive=GetDriveTypeW(wide.substr(0,3).c_str());
    Require(drive==DRIVE_FIXED||drive==DRIVE_REMOVABLE,"Network/device model paths are forbidden.");
    for(std::size_t i=0;i<wide.size();++i)
        Require(wide[i]>=32&&(wide[i]!=L':'||i==1)&&wide[i]!=L'?'&&wide[i]!=L'*',"Invalid model path.");
    std::size_t at=3;
    while(at<wide.size()) {
        const auto end=wide.find(L'\\',at); auto part=wide.substr(at,end==std::wstring::npos?end:end-at);
        Require(!part.empty()&&part!=L"."&&part!=L".."&&part.back()!=L'.'&&part.back()!=L' ',"Normalize model paths before loading.");
        part=part.substr(0,part.find(L'.'));
        for(auto& c:part) if(c>=L'a'&&c<=L'z') c=static_cast<wchar_t>(c-L'a'+L'A');
        Require(part!=L"CON"&&part!=L"PRN"&&part!=L"AUX"&&part!=L"NUL"&&part!=L"CONIN$"&&part!=L"CONOUT$"&&part!=L"CLOCK$","Reserved device model path.");
        if(part.size()==4&&(part.substr(0,3)==L"COM"||part.substr(0,3)==L"LPT"))
            Require(!((part[3]>=L'0'&&part[3]<=L'9')||part[3]==L'\x00B9'||part[3]==L'\x00B2'||part[3]==L'\x00B3'),"Reserved port model path.");
        if(end==std::wstring::npos) break; at=end+1;
    }
    struct File {HANDLE value; ~File(){if(value!=INVALID_HANDLE_VALUE)CloseHandle(value);}};
    const auto literal=L"\\\\?\\"+wide;
    File file{CreateFileW(literal.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_FLAG_OPEN_REPARSE_POINT,nullptr)};
    Require(file.value!=INVALID_HANDLE_VALUE,"Cannot open the selected model/manifest.");
    BY_HANDLE_FILE_INFORMATION info{};
    Require(GetFileType(file.value)==FILE_TYPE_DISK&&GetFileInformationByHandle(file.value,&info)
        &&!(info.dwFileAttributes&(FILE_ATTRIBUTE_REPARSE_POINT|FILE_ATTRIBUTE_DIRECTORY)),"Only regular disk model/manifest files are permitted.");
    std::vector<wchar_t> final(32768);
    const DWORD count=GetFinalPathNameByHandleW(file.value,final.data(),static_cast<DWORD>(final.size()),FILE_NAME_NORMALIZED|VOLUME_NAME_DOS);
    Require(count>=7&&count<final.size()&&std::wstring(final.data(),4)==L"\\\\?\\"
        &&final[5]==L':'&&final[6]==L'\\',"Model target must resolve to a local drive.");
    LARGE_INTEGER length{};
    Require(GetFileSizeEx(file.value,&length)&&length.QuadPart>=0&&length.QuadPart<=4*1024*1024,"Model/manifest exceeds 4 MiB.");
    std::string bytes(static_cast<std::size_t>(length.QuadPart),'\0'); std::size_t offset=0;
    while(offset<bytes.size()) {
        DWORD got=0; const auto requested=static_cast<DWORD>(std::min<std::size_t>(65536,bytes.size()-offset));
        Require(ReadFile(file.value,&bytes[offset],requested,&got,nullptr)&&got>0,"Model/manifest read failed."); offset+=got;
    } return bytes;
#else
    (void)path;
    throw ModelError(DiagnosticCode::not_implemented,"S2-B path-based model loading currently requires Windows; mathematical APIs remain portable.");
#endif
}
Json Parse(const std::string& bytes) {
    Require(bytes.size()<=4u*1024u*1024u,"Manifest byte limit exceeded.");
    std::vector<std::set<std::string>> keys(34);
    auto callback=[&](int depth,Json::parse_event_t event,Json& value) {
        Require(depth>=0&&depth<=32,"Model/manifest nesting limit exceeded.");
        if(event==Json::parse_event_t::object_start) keys[static_cast<std::size_t>(depth)+1].clear();
        if(event==Json::parse_event_t::key) Require(keys[static_cast<std::size_t>(depth)].insert(value.get<std::string>()).second,"Duplicate model/manifest key.");
        return true;};
    return Json::parse(bytes,callback,true,false);
}
void Keys(const Json& value,std::initializer_list<const char*> names) {
    Require(value.is_object()&&value.size()==names.size(),"Missing/extra manifest fields.");
    for(auto name:names) Require(value.contains(name),"Required manifest field is absent.");
}
double Number(const Json& value) {
    Require(value.is_number(),"Model values must be numeric."); const auto number=value.get<double>();
    Require(std::isfinite(number),"Model values must be finite."); return number;
}
void Count(const Json& value,std::uint64_t expected) {
    Require(value.is_number_integer()&&value==expected,"S2-B supports exactly eight cables and four rails.");
}
Eigen::Vector3d Vector(const Json& value) {
    Require(value.is_array()&&value.size()==3,"A three-component vector is required.");
    return Eigen::Vector3d(Number(value.at(0)),Number(value.at(1)),Number(value.at(2)));
}
std::string Digest(const std::string& bytes) {
    const auto hash=ComputeSourceSha256(bytes);
    Require(hash.status==ResultStatus::success,"Hashing failed.",DiagnosticCode::execution_fault); return hash.value;
}
Json Vectors(const std::array<Eigen::Vector3d,8>& values) {
    Json result=Json::array();
    for(const auto& value:values) result.push_back({value.x(),value.y(),value.z()}); return result;
}
std::string RailDigest(const RobotModelSnapshot& model) {
    Require(sizeof(double)==8&&std::numeric_limits<double>::is_iec559,"IEEE-754 binary64 is required.");
    std::string bytes="cdpr-rail-state-v1\n";
    const std::array<std::array<double,4>,2> groups{{model.rail_offsets_m,model.kinematics.startingRailOffsets}};
    for(const auto& group:groups) for(double value:group) {
        Require(std::isfinite(value),"Rail offsets must be finite."); if(value==0)value=0;
        std::uint64_t bits=0; std::memcpy(&bits,&value,sizeof(bits));
        for(int shift=56;shift>=0;shift-=8) bytes.push_back(static_cast<char>((bits>>shift)&255));
    } return Digest(bytes);
}
std::string SnapshotDigest(const RobotModelSnapshot& model) {
    const auto limits=SerializeMotionLimits(model.limits); const auto frames=SerializeFrameRegistration(model.frames);
    Require(limits.status==ResultStatus::success&&frames.status==ResultStatus::success,"Invalid model limits/registration.");
    const auto& k=model.kinematics; const auto& id=model.identity;
    Json canonical={{"contract","cu_brick_fixed_rails_v1"},{"cable_count",k.cableCount},{"rail_count",model.rail_count},
        {"radius",k.pulleyRadius},{"frame",Vectors(k.frameAttachments)},{"ee",Vectors(k.endEffectorAttachments)},{"axes",Vectors(k.pulleyAxes)},
        {"rails",model.rail_offsets_m},{"start_rails",k.startingRailOffsets},{"scales",model.cable_scale_counts_per_m},
        {"robot_hash",id.robot_config_sha256},{"calibration_hash",id.calibration_sha256},{"rail_hash",id.rail_state_sha256},{"limits_hash",id.motion_limits_sha256},
        {"has_calibration",model.has_calibration},{"has_motion_limits",model.has_motion_limits},
        {"limits",Json::parse(limits.value)},{"frames",Json::parse(frames.value)}};
    return Digest(canonical.dump());
}
void Structure(const RobotModelSnapshot& model) {
    Require(model.kinematics.cableCount==8&&model.rail_count==4,"S2-B requires eight cables/four rails.");
    Require(std::isfinite(model.kinematics.pulleyRadius)&&model.kinematics.pulleyRadius>0,"Invalid pulley radius.");
    for(std::size_t i=0;i<8;++i) {
        Require(model.kinematics.frameAttachments[i].allFinite()&&model.kinematics.endEffectorAttachments[i].allFinite()
            &&model.kinematics.pulleyAxes[i].allFinite(),"Nonfinite cable geometry.");
        const auto& axis=model.kinematics.pulleyAxes[i];
        Require(axis.x()==0&&axis.y()==0&&std::abs(axis.z())==1,"Legacy pulley axes must be unit +/-Z.");
        Require(std::isfinite(model.cable_scale_counts_per_m[i])&&model.cable_scale_counts_per_m[i]!=0,"Missing/invalid cable count scale.");
    }
    for(std::size_t i=0;i<4;++i) Require(std::isfinite(model.rail_offsets_m[i])&&std::isfinite(model.kinematics.startingRailOffsets[i]),"Missing/invalid rail state.");
    Require(model.identity.robot_config_sha256.size()==64&&model.identity.rail_state_sha256==RailDigest(model),"Rail/model identity mismatch.",DiagnosticCode::identity_mismatch);
    Require(model.has_calibration?model.identity.calibration_sha256.size()==64&&model.frames.status==RegistrationStatus::verified
        :model.identity.calibration_sha256.empty()&&model.frames.status==RegistrationStatus::unverified,"Calibration provenance mismatch.");
    Require(model.has_motion_limits?model.identity.motion_limits_sha256.size()==64:model.identity.motion_limits_sha256.empty(),"Limit provenance mismatch.");
}
void MergeControllerBounds(MotionLimits& proposed,const MotionLimits& controller) {
    if(controller.workspace.has_value) {
        if(proposed.workspace.has_value) for(std::size_t i=0;i<3;++i)
            Require(proposed.workspace.value.minimum_m[i]>=controller.workspace.value.minimum_m[i]
                &&proposed.workspace.value.maximum_m[i]<=controller.workspace.value.maximum_m[i],"A planning manifest cannot widen controller XYZ bounds.");
        else proposed.workspace=controller.workspace;
    }
    if(controller.maximum_path_speed_m_per_s.has_value) {
        if(proposed.maximum_path_speed_m_per_s.has_value)
            Require(proposed.maximum_path_speed_m_per_s.value<=controller.maximum_path_speed_m_per_s.value,"A planning manifest cannot raise the controller path-speed bound.");
        else proposed.maximum_path_speed_m_per_s=controller.maximum_path_speed_m_per_s;
    }
}
} // namespace

Result<RobotModelSnapshot> LoadRobotModel(const ModelLoadRequest& request) {
    auto result=Attempt<RobotModelSnapshot>([&] {
        Require(request.rail_offsets_m.has_value&&request.starting_rail_offsets_m.has_value,"Explicit current and starting rail offsets are required.",DiagnosticCode::missing_model_data);
        const auto bytes=ReadFileBytes(request.robot_config_path); const auto root=Parse(bytes);
        Require(root.is_object(),"Robot model must be a JSON object.");
        RobotModelSnapshot model; model.identity.robot_config_sha256=Digest(bytes);
        if(request.expected_robot_config_sha256.has_value)
            Require(request.expected_robot_config_sha256.value==model.identity.robot_config_sha256,"Robot configuration digest mismatch.",DiagnosticCode::identity_mismatch);
        auto& k=model.kinematics; k.cableCount=8; model.rail_count=4;
        model.rail_offsets_m=request.rail_offsets_m.value; k.startingRailOffsets=request.starting_rail_offsets_m.value;
        if(root.contains("fixture_schema_version")) {
            Keys(root,{"fixture_schema_version","synthetic","hardware_access","purpose","kinematics"});
            Require(root.at("fixture_schema_version").is_number_integer()&&root.at("fixture_schema_version")==1
                &&root.at("synthetic").is_boolean()&&root.at("synthetic")==true
                &&root.at("hardware_access").is_boolean()&&root.at("hardware_access")==false,"Invalid synthetic fixture wrapper.");
            const auto& g=root.at("kinematics");
            Keys(g,{"cable_count","rail_count","pulley_radius_m","frame_attachments_m","end_effector_attachments_m","pulley_axes",
                "rail_offsets_m","starting_rail_offsets_m","cable_scale_counts_per_m","workspace_m"});
            Count(g.at("cable_count"),8); Count(g.at("rail_count"),4); k.pulleyRadius=Number(g.at("pulley_radius_m"));
            for(auto name:{"frame_attachments_m","end_effector_attachments_m","pulley_axes","cable_scale_counts_per_m"})
                Require(g.at(name).is_array()&&g.at(name).size()==8,"Invalid synthetic cable array.");
            for(std::size_t i=0;i<8;++i) {
                k.frameAttachments[i]=Vector(g.at("frame_attachments_m").at(i)); k.endEffectorAttachments[i]=Vector(g.at("end_effector_attachments_m").at(i));
                k.pulleyAxes[i]=Vector(g.at("pulley_axes").at(i)); model.cable_scale_counts_per_m[i]=Number(g.at("cable_scale_counts_per_m").at(i));
            }
            for(auto name:{"rail_offsets_m","starting_rail_offsets_m"}) Require(g.at(name).is_array()&&g.at(name).size()==4,"Invalid synthetic rail array.");
            for(std::size_t i=0;i<4;++i) Require(Number(g.at("rail_offsets_m").at(i))==model.rail_offsets_m[i]
                &&Number(g.at("starting_rail_offsets_m").at(i))==k.startingRailOffsets[i],"Fixture/request rail states disagree.");
            const auto empty=SerializeMotionLimits(MotionLimits{}); Require(empty.status==ResultStatus::success,"Limit encoding failed.");
            auto limits=Json::parse(empty.value); limits["workspace"]=g.at("workspace_m");
            const auto decoded=DeserializeMotionLimits(limits.dump()); Require(decoded.status==ResultStatus::success,"Invalid synthetic workspace.");
            model.limits=decoded.value;
        } else {
            Count(root.at("cableMotorNum"),8); Count(root.at("railMotorNum"),4); k.pulleyRadius=Number(root.at("pulleyRadius"));
            for(auto name:{"frameAttachments","endEffectorAttachments","pulleyZdir"})
                Require(root.at(name).is_array()&&root.at(name).size()==8,"Robot geometry arrays must have eight entries.");
            bool individual=false;
            if(root.contains("useIndividualCableScale")) {
                Require(root.at("useIndividualCableScale").is_boolean(),"useIndividualCableScale must be boolean.");
                individual=root.at("useIndividualCableScale").get<bool>();
            }
            if(individual) Require(root.at("cableMotorScaleIndividual").is_array()&&root.at("cableMotorScaleIndividual").size()==8,"Individual scales are enabled but absent/invalid.");
            const double raw_common=individual?0:Number(root.at("cableMotorScale"));
            Require(individual||(raw_common>=std::numeric_limits<int>::min()&&raw_common<=std::numeric_limits<int>::max()),
                "Common scale is outside Robot's integer JSON conversion range.");
            // Robot uses model.value("cableMotorScale", 509295); its integer
            // default makes nlohmann return int, even for a fractional JSON value.
            const double common=individual?0:static_cast<double>(static_cast<int>(raw_common));
            for(std::size_t i=0;i<8;++i) {
                k.frameAttachments[i]=Vector(root.at("frameAttachments").at(i)); k.endEffectorAttachments[i]=Vector(root.at("endEffectorAttachments").at(i));
                k.pulleyAxes[i]=Eigen::Vector3d(0,0,Number(root.at("pulleyZdir").at(i)));
                // Match Robot::LoadModel's explicit float conversion only in individual mode.
                model.cable_scale_counts_per_m[i]=individual?static_cast<double>(static_cast<float>(Number(root.at("cableMotorScaleIndividual").at(i)))):common;
            }
            const auto& b=root.at("endEffectorPosLimit"); const char* labels[]={"x","y","z","roll","pitch","yaw"};
            model.limits.workspace.has_value=true;
            for(std::size_t i=0;i<3;++i) {
                model.limits.workspace.value.minimum_m[i]=static_cast<double>(static_cast<float>(Number(b.at(labels[2*i]))));
                model.limits.workspace.value.maximum_m[i]=static_cast<double>(static_cast<float>(Number(b.at(labels[2*i+1]))));
            }
            if(root.contains("endEffVelLmt")) {
                model.limits.maximum_path_speed_m_per_s.has_value=true;
                model.limits.maximum_path_speed_m_per_s.value=static_cast<double>(static_cast<float>(Number(root.at("endEffVelLmt"))));
            }
        }
        const auto geometry_limits=SerializeMotionLimits(model.limits);
        Require(geometry_limits.status==ResultStatus::success,"Controller/synthetic bounds are invalid.");
        if(request.calibration_manifest_path.has_value) {
            const auto calibration_bytes=ReadFileBytes(request.calibration_manifest_path.value); const auto calibration=Parse(calibration_bytes);
            Keys(calibration,{"schema_version","frames","provenance"});
            Require(calibration.at("schema_version").is_number_integer()&&calibration.at("schema_version")==1,"Unsupported calibration manifest version.");
            const auto& provenance=calibration.at("provenance"); Keys(provenance,{"calibration_id","method"});
            for(auto name:{"calibration_id","method"}) Require(provenance.at(name).is_string()
                &&!provenance.at(name).get<std::string>().empty()&&provenance.at(name).get<std::string>().size()<=256,"Calibration provenance is required.");
            const auto frames=DeserializeFrameRegistration(calibration.at("frames").dump());
            Require(frames.status==ResultStatus::success&&frames.value.status==RegistrationStatus::verified,"Calibration requires registered transforms.");
            model.frames=frames.value; model.has_calibration=true; model.identity.calibration_sha256=Digest(calibration_bytes);
        }
        if(request.motion_limits_manifest_path.has_value) {
            const auto limit_bytes=ReadFileBytes(request.motion_limits_manifest_path.value); const auto manifest=Parse(limit_bytes);
            Keys(manifest,{"schema_version","geometry_contract","limits"});
            Require(manifest.at("schema_version").is_number_integer()&&manifest.at("schema_version")==1
                &&manifest.at("geometry_contract").is_string()&&manifest.at("geometry_contract")=="cu_brick_fixed_rails_v1","Unsupported limits/geometry contract.");
            const auto limits=DeserializeMotionLimits(manifest.at("limits").dump()); Require(limits.status==ResultStatus::success,"Invalid motion-limits manifest.");
            auto merged=limits.value; MergeControllerBounds(merged,model.limits); model.limits=merged;
            model.has_motion_limits=true; model.identity.motion_limits_sha256=Digest(limit_bytes);
        }
        model.identity.rail_state_sha256=RailDigest(model); Structure(model);
        model.snapshot_sha256=SnapshotDigest(model); return model;
    });
    if(result.status==ResultStatus::success&&(!result.value.has_calibration||!result.value.has_motion_limits)) {
        Diagnostic d; d.severity=Severity::warning; d.code=DiagnosticCode::missing_model_data;
        d.message="Provisional geometry snapshot only: calibration and/or limits manifest is missing."; result.diagnostics.push_back(d);
    }
    return result;
}

Result<ValidationReport> ValidateModelSnapshot(const RobotModelSnapshot& model) {
    return Attempt<ValidationReport>([&] {
        Structure(model);
        Require(!model.snapshot_sha256.empty()&&model.snapshot_sha256==SnapshotDigest(model),"Decoded model snapshot was changed or was not loaded by the strict loader.",DiagnosticCode::identity_mismatch);
        return ValidationReport{};
    });
}
Result<PlanningOptions> BindPlanningOptions(const PlanningOptions& options,const RobotModelSnapshot& model) {
    return Attempt<PlanningOptions>([&] {
        const auto valid=ValidateModelSnapshot(model); Require(valid.status==ResultStatus::success,"Invalid/stale model snapshot.",DiagnosticCode::identity_mismatch);
        Require(model.has_calibration&&model.has_motion_limits,"Calibration and limits manifests are required for planning.",DiagnosticCode::missing_model_data);
        Require(options.fixed_orientation&&options.fixed_rails,"S2-B requires fixed orientation and fixed rails.");
        Require(options.maximum_samples>=5&&options.maximum_samples<=200000&&options.maximum_samples_per_motion>=5
            &&options.maximum_samples_per_motion<=50000&&options.maximum_samples_per_motion<=options.maximum_samples,"Invalid sample resource bounds.");
        PlanningOptions bound=options;
        const auto a=SerializeFrameRegistration(options.frames), b=SerializeFrameRegistration(model.frames);
        Require(a.status==ResultStatus::success&&b.status==ResultStatus::success,"Invalid planning frame registration.");
        const auto given=Json::parse(a.value), actual=Json::parse(b.value);
        if(!options.frames.world_from_bed.has_value&&!options.frames.end_effector_from_nozzle.has_value&&options.frames.status==RegistrationStatus::unverified) {
            for(auto key:{"world_frame_id","bed_frame_id","end_effector_frame_id","nozzle_frame_id"})
                Require(given.at(key)==actual.at(key),"Configured/manifest frame names disagree.");
        } else Require(given==actual,"Configured transforms disagree with calibration manifest.",DiagnosticCode::identity_mismatch);
        const auto supplied_limits=SerializeMotionLimits(options.limits), model_limits=SerializeMotionLimits(model.limits);
        Require(supplied_limits.status==ResultStatus::success&&model_limits.status==ResultStatus::success,"Invalid planning limits.");
        const auto supplied=Json::parse(supplied_limits.value), recorded=Json::parse(model_limits.value);
        for(auto item=supplied.begin();item!=supplied.end();++item)
            if(!item.value().is_null()) Require(item.value()==recorded.at(item.key()),"Configured limits disagree with the bound model/manifest.",DiagnosticCode::identity_mismatch);
        if(options.model_identity.has_value) {
            const auto& x=options.model_identity.value; const auto& y=model.identity;
            Require(x.robot_config_sha256==y.robot_config_sha256&&x.calibration_sha256==y.calibration_sha256
                &&x.rail_state_sha256==y.rail_state_sha256&&x.motion_limits_sha256==y.motion_limits_sha256,"Planning/model identity mismatch.",DiagnosticCode::identity_mismatch);
        }
        bound.frames=model.frames; bound.limits=model.limits; bound.model_identity.has_value=true; bound.model_identity.value=model.identity; return bound;
    });
}
} // namespace cdpr_slicer
