#include "cdpr_slicer/Mechanics.h"
#include "json.hpp"
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace cdpr_slicer;
using Json=nlohmann::json;
namespace {
void Check(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
std::string Read(const std::string& path) {std::ifstream file(path);Check(file.good(),"Cannot open mechanics test fixture.");std::ostringstream bytes;bytes<<file.rdbuf();return bytes.str();}
RobotModelSnapshot Model() {
    const std::string root=CDPR_SLICER_PROJECT_DIRECTORY;ModelLoadRequest request;
    request.robot_config_path=root+"/tests/fixtures/robot-model.json";
    request.calibration_manifest_path.has_value=true;request.calibration_manifest_path.value=root+"/config/synthetic-calibration.json";
    request.motion_limits_manifest_path.has_value=true;request.motion_limits_manifest_path.value=root+"/config/synthetic-limits.json";
    request.rail_offsets_m.has_value=true;request.rail_offsets_m.value={{0,0,0,0}};request.starting_rail_offsets_m=request.rail_offsets_m;
    const auto loaded=LoadRobotModel(request);Check(loaded.status==ResultStatus::success,"Mechanics fixture model failed.");return loaded.value;
}
PrintJob Job(const RobotModelSnapshot& model) {
    GCodeParseRequest request;request.text="G21\nG90\nM82\nG1 X10 E1 F600\n";request.source.file_name="mechanics-test.gcode";
    auto& initial=request.options.initial_state;initial.nozzle_pose_in_bed.has_value=true;initial.logical_e_mm.has_value=true;initial.filament_position_mm.has_value=true;
    const auto parsed=ParseGCode(request);Check(parsed.status==ResultStatus::success,"Mechanics fixture import failed.");
    const auto options=BindPlanningOptions(PlanningOptions{},model);Check(options.status==ResultStatus::success,"Mechanics fixture options failed.");
    const auto transformed=TransformToolpath(parsed.value,options.value);Check(transformed.status==ResultStatus::success,"Mechanics fixture transform failed.");return transformed.value;
}
Json Physics(const RobotModelSnapshot& model) {
    return {{"schema_version",1},{"model_id","unit-test-only"},{"provenance","Synthetic mathematical test, never AB1 measurements"},{"synthetic",true},{"robot_config_sha256",model.identity.robot_config_sha256},
        {"sampling",{{"maximum_step_m",0.005},{"maximum_samples",1000},{"maximum_samples_per_motion",1000}}},
        {"collision",nullptr},
        {"statics",{{"mass_kg",2},{"cog_ee_m",{0,0,0}},{"gravity_m_per_s2",{0,0,-9.80665}},{"external_force_n",{0,0,0}},{"external_torque_nm",{0,0,0}},{"minimum_tension_n",1},{"maximum_tension_n",100},{"breaking_strength_n",nullptr}}}};
}
Json Collision() {
    return {{"cable_radius_m",0.001},{"clearance_m",0},{"attachment_exclusion_m",0.4},{"ee_boxes_local_m",nullptr},{"obstacles_world",Json::array()}};
}
Json Report(const PrintJob& job,const RobotModelSnapshot& model,const Json& physical) {
    const auto assessed=AssessMechanics(job,model,physical.dump());
    if(assessed.status!=ResultStatus::success) {if(!assessed.diagnostics.empty())std::cerr<<assessed.diagnostics[0].message<<'\n';throw std::runtime_error("Expected complete mechanics report.");}
    Check(ValidateMechanicsAssessment(job,model,assessed.value).status==ResultStatus::success,"Generated mechanics report rejected itself.");return Json::parse(assessed.value);
}
void MissingAndBinding() {
    const auto model=Model();const auto job=Job(model);auto spec=Physics(model);spec["statics"]=nullptr;
    const auto before=SerializePrintJob(job).value;const auto report=Report(job,model,spec);
    Check(report["sample_count"]==0&&report["collision"]["status"]=="not_evaluated"&&report["tension"]["status"]=="not_evaluated","Missing physical data fabricated a result.");
    Check(SerializePrintJob(job).value==before,"Mechanics changed the planning artifact.");
    spec["robot_config_sha256"]=std::string(64,'f');Check(AssessMechanics(job,model,spec.dump()).status==ResultStatus::rejected,"Physical model for another robot was accepted.");
    auto altered=report;altered["hardware_ready"]=true;Check(ValidateMechanicsAssessment(job,model,altered.dump()).status==ResultStatus::rejected,"Mechanics readiness tampering was accepted.");
}
void EquilibriumAndUnknownLimits() {
    const auto model=Model();const auto job=Job(model);auto spec=Physics(model);spec["statics"]["cog_ee_m"]={0.01,0,0};const auto report=Report(job,model,spec);
    Check(report["tension"]["status"]=="feasible_at_samples","Supported synthetic gravity load failed.");
    for(const auto& sample:report["samples"]) {
        const auto id=sample["operation_id"].get<std::size_t>();auto pose=job.operations[id].motion.value.end_effector.value.start;
        for(int k=0;k<3;++k) pose.position_m[k]=sample["ee_world_m"][k];
        const auto spans=ComputeCheckedCableSpans(model,ToLegacyPose(pose).value);Check(spans.status==ResultStatus::success,"Free span computation failed.");
        Eigen::Vector3d force(0,0,-2*9.80665),moment=Eigen::Vector3d(.01,0,0).cross(force);
        const Eigen::Vector3d origin(pose.position_m[0],pose.position_m[1],pose.position_m[2]);
        for(int i=0;i<8;++i) {
            const auto& span=spans.value[i];const Eigen::Vector3d a(span.attachment_m[0],span.attachment_m[1],span.attachment_m[2]),b(span.pulley_m[0],span.pulley_m[1],span.pulley_m[2]);
            const double tension=sample["tension_n"][i];Check(tension>=1-1e-8&&tension<=100+1e-8,"Tension witness violated its bounds.");const Eigen::Vector3d cable_force=(b-a).normalized()*tension;force+=cable_force;moment+=(a-origin).cross(cable_force);
        }
        Check(force.norm()<1e-6&&moment.norm()<1e-6,"Independent force/moment balance failed.");
    }
    spec["statics"]["minimum_tension_n"]=nullptr;spec["statics"]["maximum_tension_n"]=nullptr;const auto unknown=Report(job,model,spec);
    Check(unknown["tension"]["status"]=="limits_not_evaluated"&&!unknown["samples"][0]["tension_n"].is_null(),"Unknown working limits were fabricated or hid the equilibrium allocation.");
    spec["statics"]["breaking_strength_n"]=1700;const auto ultimate=Report(job,model,spec);
    Check(ultimate["tension"]["status"]=="limits_not_evaluated"&&ultimate["tension"]["breaking_strength_known"]==true,"Breaking strength was promoted to a working limit.");
}
void InfeasibleAndRank() {
    const auto model=Model();const auto job=Job(model);auto spec=Physics(model);spec["statics"]["mass_kg"]=1000;spec["statics"]["maximum_tension_n"]=5;
    const auto report=Report(job,model,spec);Check(report["tension"]["status"]=="infeasible_at_samples"&&report["samples"][0]["tension_n"].is_null(),"Overloaded fixture returned a force allocation.");
    spec=Physics(model);spec["statics"]["external_torque_nm"]={0,0,1};const auto yaw=Report(job,model,spec);
    Check(yaw["tension"]["minimum_wrench_rank"].get<int>()<6&&yaw["tension"]["status"]=="infeasible_at_samples","Uncontrollable wrench was treated as supportable.");
}
void CollisionCoverage() {
    const auto model=Model();const auto job=Job(model);auto spec=Physics(model);spec["statics"]=nullptr;spec["collision"]=Collision();
    Check(Report(job,model,spec)["collision"]["status"]=="sampled_clear","Separated trimmed free spans collided.");
    const auto pose=job.operations[0].motion.value.end_effector.value.start;
    const auto spans=ComputeCheckedCableSpans(model,ToLegacyPose(pose).value);Check(spans.status==ResultStatus::success,"Cannot sample collision fixture.");
    Json lo=Json::array(),hi=Json::array();for(int k=0;k<3;++k) {const double mid=(spans.value[0].pulley_m[k]+spans.value[0].attachment_m[k])/2;lo.push_back(mid-.001);hi.push_back(mid+.001);}
    spec["collision"]["obstacles_world"]=Json::array({{{"id","thin_midspan_obstacle"},{"minimum_m",lo},{"maximum_m",hi}}});
    auto report=Report(job,model,spec);bool midspan=false;for(const auto& c:report["samples"][0]["contacts"]) if(c["kind"]=="cable_obstacle") midspan=true;
    Check(report["collision"]["status"]=="potential_collision"&&midspan,"Segment-interior obstacle was missed.");
    spec["collision"]=Collision();spec["collision"]["ee_boxes_local_m"]=Json::array({{{"id","small_body"},{"minimum_m",{-.01,-.01,-.01}},{"maximum_m",{.01,.01,.01}}}});
    lo=Json::array();hi=Json::array();for(int k=0;k<3;++k){lo.push_back(pose.position_m[k]-.001);hi.push_back(pose.position_m[k]+.001);}
    spec["collision"]["obstacles_world"]=Json::array({{{"id","body_obstacle"},{"minimum_m",lo},{"maximum_m",hi}}});report=Report(job,model,spec);bool body=false;for(const auto& c:report["samples"][0]["contacts"]) if(c["kind"]=="body_obstacle") body=true;
    Check(body,"Body-obstacle overlap was missed.");
    spec["collision"]["obstacles_world"]=Json::array();
    lo=Json::array({pose.position_m[0]-.1,pose.position_m[1]-.1,pose.position_m[2]});
    hi=Json::array({pose.position_m[0]+.1,pose.position_m[1]+.1,pose.position_m[2]});
    spec["collision"]["bed_reference_world"]={{"id","test_bed_plane"},{"minimum_m",lo},{"maximum_m",hi}};
    Check(Report(job,model,spec)["collision"]["status"]=="potential_collision","Zero-thickness bed reference crossing was missed.");
    spec["collision"]=Collision();spec["collision"]["attachment_exclusion_m"]=0;report=Report(job,model,spec);bool pair=false;for(const auto& c:report["samples"][0]["contacts"]) if(c["kind"]=="cable_cable") pair=true;
    Check(pair,"Crossing free cable pair was missed.");
    spec["collision"]["ee_boxes_local_m"]=Json::array();
    for(int i=0;i<8;++i) {
        const auto& a=model.kinematics.endEffectorAttachments[i];lo=Json::array();hi=Json::array();
        for(int k=0;k<3;++k){lo.push_back(a[k]-.001);hi.push_back(a[k]+.001);}
        spec["collision"]["ee_boxes_local_m"].push_back({{"id","mount-"+std::to_string(i)},{"minimum_m",lo},{"maximum_m",hi}});
    }
    report=Report(job,model,spec);const auto& contact_sample=report["samples"][0];
    Check(contact_sample["collision_count"].get<int>()>4&&contact_sample["contacts"].size()==4,"Crowded contact fixture did not exercise the reporting cap.");
    for(const auto& c:contact_sample["contacts"])Check(c["kind"]=="cable_cable","Endpoint contacts hid a closer crossing from the report.");
}
void GuardsAndPreview() {
    const auto model=Model();const auto job=Job(model);auto spec=Physics(model);spec["sampling"]["maximum_samples"]=2;spec["sampling"]["maximum_samples_per_motion"]=2;
    const auto exhausted=AssessMechanics(job,model,spec.dump());Check(exhausted.status==ResultStatus::rejected&&exhausted.value.empty(),"Sample exhaustion exposed a partial report.");
    spec=Physics(model);spec["statics"]["mass_kg"]=-1;Check(AssessMechanics(job,model,spec.dump()).status==ResultStatus::rejected,"Negative mass was accepted.");
    spec=Physics(model);spec["statics"]["maximum_tension_n"]=0;Check(AssessMechanics(job,model,spec.dump()).status==ResultStatus::rejected,"Invalid force bounds were accepted.");
    spec=Physics(model);const auto report=Report(job,model,spec);const auto html=CreatePreviewHtml(job,model,report.dump());Check(html.status==ResultStatus::success&&html.value.find("Sampled mechanics")!=std::string::npos,"Mechanics preview failed.");
    auto changed=report;changed["samples"][0]["tension_n"][0]=999;Check(CreatePreviewHtml(job,model,changed.dump()).status==ResultStatus::rejected,"Changed report was attached to preview.");
    spec["provenance"]="</script><script>window.injected=true</script>";const auto inert=Report(job,model,spec);const auto safe=CreatePreviewHtml(job,model,inert.dump());Check(safe.status==ResultStatus::success&&safe.value.find("</script><script>window.injected=true")==std::string::npos,"Mechanics model text escaped its inert block.");
    auto other=job;other.source.file_name="different.gcode";Check(ValidateMechanicsAssessment(other,model,report.dump()).status==ResultStatus::rejected,"Stale report was accepted for a changed job.");
    auto duplicate=spec.dump();duplicate.insert(1,"\"schema_version\":1,");Check(AssessMechanics(job,model,duplicate).status==ResultStatus::rejected,"Duplicate physical JSON keys were accepted.");
}
} // namespace
int main() {
    struct Case {const char* name;void(*run)();};const Case cases[]={{"missing data and robot binding",MissingAndBinding},{"bounded equilibrium and unknown limits",EquilibriumAndUnknownLimits},{"infeasibility and rank",InfeasibleAndRank},{"modeled collision coverage",CollisionCoverage},{"budgets, stale reports and preview escaping",GuardsAndPreview}};
    try{for(const auto& test:cases){test.run();std::cout<<"PASS "<<test.name<<'\n';}}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}return 0;
}
