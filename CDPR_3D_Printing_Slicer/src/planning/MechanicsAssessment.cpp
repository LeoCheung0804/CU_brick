#include "cdpr_slicer/Mechanics.h"
#include "json.hpp"
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace cdpr_slicer {
namespace {
using Json=nlohmann::json;
using V=Eigen::Vector3d;
struct Error:std::runtime_error { using std::runtime_error::runtime_error; };
void Need(bool condition,const char* message) { if(!condition) throw Error(message); }
std::string Hash(const std::string& text) {
    const auto hash=ComputeSourceSha256(text); Need(hash.status==ResultStatus::success,"Cannot hash assessment data."); return hash.value;
}
Json Parse(const std::string& text) {
    Need(text.size()<=24u*1024u*1024u,"Mechanics JSON exceeds 24 MiB.");
    std::vector<std::set<std::string>> keys(34);
    auto callback=[&](int depth,Json::parse_event_t event,Json& value) {
        Need(depth>=0&&depth<=32,"Mechanics JSON nesting limit exceeded.");
        if(event==Json::parse_event_t::object_start) keys[static_cast<std::size_t>(depth)+1].clear();
        if(event==Json::parse_event_t::key) Need(keys[static_cast<std::size_t>(depth)].insert(value.get<std::string>()).second,"Duplicate mechanics JSON key.");
        return true;
    };
    return Json::parse(text,callback,true,false);
}
void Keys(const Json& object,std::initializer_list<const char*> names) {
    Need(object.is_object()&&object.size()==names.size(),"Missing/extra mechanics fields.");
    for(auto name:names) Need(object.contains(name),"Missing mechanics field.");
}
double Number(const Json& value) {
    Need(value.is_number(),"A finite mechanics number is required.");const double n=value.get<double>();
    Need(std::isfinite(n)&&std::abs(n)<=1e9,"Mechanics number exceeds the numeric envelope.");return n;
}
V Vec(const Json& value) {
    Need(value.is_array()&&value.size()==3,"Expected a three-element mechanics vector.");
    return V(Number(value[0]),Number(value[1]),Number(value[2]));
}
V Vec(const std::array<double,3>& value) { return V(value[0],value[1],value[2]); }
Json J(const V& value) { return Json::array({value.x(),value.y(),value.z()}); }
Eigen::Matrix3d Rotation(const Quaternion& q) {
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    Need(rotation.coeffs().allFinite()&&std::abs(rotation.squaredNorm()-1)<1e-10,"Invalid mechanics pose quaternion.");
    return rotation.toRotationMatrix();
}
struct Box { V lo,hi;std::string id; };
Box ReadBox(const Json& value,bool named,bool reference_plane=false) {
    if(named) Keys(value,{"id","minimum_m","maximum_m"}); else Keys(value,{"minimum_m","maximum_m"});
    Box b{Vec(value.at("minimum_m")),Vec(value.at("maximum_m")),"end_effector"};
    Need(reference_plane?(b.hi.x()>b.lo.x()&&b.hi.y()>b.lo.y()&&b.hi.z()==b.lo.z()):(b.hi.array()>b.lo.array()).all(),"Collision boxes require positive dimensions; bed reference is a positive-area XY plane.");
    if(named) {Need(value.at("id").is_string(),"Obstacle id must be a string.");b.id=value.at("id").get<std::string>();Need(!b.id.empty()&&b.id.size()<=80,"Invalid obstacle id.");}
    return b;
}
struct Spec {
    Json json; double step;std::size_t maximum,per_motion;
    bool collision=false,body=false,environment=false,bed_reference=false,statics=false,lower_known=false,upper_known=false,breaking_known=false,upper_cap=false;
    double radius=0,clearance=0,exclusion=0,mass=0;
    std::vector<Box> bodies,obstacles;
    V cog=V::Zero(),gravity=V::Zero(),force=V::Zero(),torque=V::Zero();
    Eigen::Matrix<double,8,1> lower,upper,breaking;
};
Spec ReadSpec(const Json& value) {
    Keys(value,{"schema_version","model_id","provenance","synthetic","robot_config_sha256","sampling","collision","statics"});
    Need(value.at("schema_version").is_number_integer()&&value.at("schema_version")==1,"Unsupported mechanics model schema.");
    for(auto field:{"model_id","provenance"}) Need(value.at(field).is_string()&&!value.at(field).get<std::string>().empty()&&value.at(field).get<std::string>().size()<=1024,"Mechanics model identity/provenance is required.");
    Need(value.at("synthetic").is_boolean(),"An explicit synthetic flag is required.");
    Need(value.at("robot_config_sha256").is_string()&&value.at("robot_config_sha256").get<std::string>().size()==64,"An explicit robot configuration digest is required for the physical model.");
    Spec s;s.json=value;const auto& sampling=value.at("sampling");Keys(sampling,{"maximum_step_m","maximum_samples","maximum_samples_per_motion"});
    s.step=Number(sampling.at("maximum_step_m"));Need(s.step>0&&s.step<=1,"Spatial mechanics step must be in (0,1] metres.");
    Need(sampling.at("maximum_samples").is_number_integer()&&sampling.at("maximum_samples_per_motion").is_number_integer(),"Mechanics sample budgets must be integers.");
    const auto maximum=sampling.at("maximum_samples").get<long long>(),per=sampling.at("maximum_samples_per_motion").get<long long>();
    Need(maximum>=1&&maximum<=20000&&per>=2&&per<=2001&&per<=maximum,"Invalid mechanics sample budgets.");
    s.maximum=static_cast<std::size_t>(maximum);s.per_motion=static_cast<std::size_t>(per);
    if(!value.at("collision").is_null()) {
        const auto& c=value.at("collision");
        if(c.contains("bed_reference_world")) Keys(c,{"cable_radius_m","clearance_m","attachment_exclusion_m","ee_boxes_local_m","obstacles_world","bed_reference_world"});
        else Keys(c,{"cable_radius_m","clearance_m","attachment_exclusion_m","ee_boxes_local_m","obstacles_world"});
        s.collision=true;s.radius=Number(c.at("cable_radius_m"));s.clearance=Number(c.at("clearance_m"));s.exclusion=Number(c.at("attachment_exclusion_m"));
        Need(s.radius>0&&s.radius<=.1&&s.clearance>=0&&s.clearance<=1&&s.exclusion>=0&&s.exclusion<=.5,"Invalid collision radius/clearance/exclusion.");
        if(!c.at("ee_boxes_local_m").is_null()) {
            Need(c.at("ee_boxes_local_m").is_array()&&!c.at("ee_boxes_local_m").empty()&&c.at("ee_boxes_local_m").size()<=32,"Provide 1..32 EE component boxes, or null for unknown geometry.");
            s.body=true;std::set<std::string> ids;
            for(const auto& item:c.at("ee_boxes_local_m")) {auto b=ReadBox(item,true);Need(ids.insert(b.id).second,"Duplicate EE component id.");s.bodies.push_back(std::move(b));}
        }
        if(!c.at("obstacles_world").is_null()) {
            Need(c.at("obstacles_world").is_array()&&c.at("obstacles_world").size()<=64,"At most 64 obstacle boxes are supported.");
            s.environment=true;std::set<std::string> ids;
            for(const auto& item:c.at("obstacles_world")) {auto b=ReadBox(item,true);Need(ids.insert(b.id).second,"Duplicate obstacle id.");s.obstacles.push_back(std::move(b));}
        }
        if(c.contains("bed_reference_world")&&!c.at("bed_reference_world").is_null()) {
            auto plane=ReadBox(c.at("bed_reference_world"),true,true);for(const auto& box:s.obstacles)Need(box.id!=plane.id,"Duplicate bed/obstacle id.");
            s.obstacles.push_back(std::move(plane));s.bed_reference=true;
        }
    }
    if(!value.at("statics").is_null()) {
        const auto& t=value.at("statics");Keys(t,{"mass_kg","cog_ee_m","gravity_m_per_s2","external_force_n","external_torque_nm","minimum_tension_n","maximum_tension_n","breaking_strength_n"});
        s.statics=true;s.mass=Number(t.at("mass_kg"));Need(s.mass>0&&s.mass<=100000,"Moving mass must be positive and <=100000 kg.");
        s.cog=Vec(t.at("cog_ee_m"));s.gravity=Vec(t.at("gravity_m_per_s2"));s.force=Vec(t.at("external_force_n"));s.torque=Vec(t.at("external_torque_nm"));
        s.lower_known=!t.at("minimum_tension_n").is_null();s.upper_known=!t.at("maximum_tension_n").is_null();
        s.breaking_known=!t.at("breaking_strength_n").is_null();s.upper_cap=s.upper_known||s.breaking_known;
        for(int i=0;i<8;++i) {
            auto bound=[&](const char* field) {const auto& b=t.at(field);if(b.is_number()) return Number(b);Need(b.is_array()&&b.size()==8,"Tension bounds must be one common number or eight numbers.");return Number(b[i]);};
            s.lower[i]=s.lower_known?bound("minimum_tension_n"):0;
            s.breaking[i]=s.breaking_known?bound("breaking_strength_n"):std::numeric_limits<double>::infinity();
            s.upper[i]=s.upper_known?bound("maximum_tension_n"):s.breaking[i];
            Need(s.lower[i]>=0&&s.lower[i]<=1e7&&s.upper[i]>s.lower[i]&&(!s.upper_cap||s.upper[i]<=1e7),"Invalid cable tension bounds in newtons.");
            Need(!s.breaking_known||(s.breaking[i]>0&&s.breaking[i]<=1e7&&s.upper[i]<=s.breaking[i]),"Working tension cannot exceed the supplied breaking strength.");
        }
    }
    return s;
}
double SegmentDistance(const V& p,const V& q,const V& a,const V& b) {
    const V u=q-p,v=b-a,w=p-a;const double aa=u.squaredNorm(),bb=u.dot(v),cc=v.squaredNorm(),dd=u.dot(w),ee=v.dot(w);
    double s=0,t=0;
    if(aa<1e-24&&cc<1e-24) return w.norm();
    if(aa<1e-24) t=std::max(0.0,std::min(1.0,ee/cc));
    else {
        if(cc<1e-24) s=std::max(0.0,std::min(1.0,-dd/aa));
        else {
            const V normal=u.cross(v);const double denominator=normal.squaredNorm();
            if(denominator>0) s=std::max(0.0,std::min(1.0,(a-p).cross(v).dot(normal)/denominator));
            t=(bb*s+ee)/cc;
            if(t<0) {t=0;s=std::max(0.0,std::min(1.0,-dd/aa));}
            else if(t>1) {t=1;s=std::max(0.0,std::min(1.0,(bb-dd)/aa));}
        }
    }
    return (w+s*u-t*v).norm();
}
double PointBox(const V& p,const Box& b) {
    return (b.lo-p).cwiseMax(V::Zero()).cwiseMax(p-b.hi).norm();
}
double SegmentBox(const V& a,const V& b,const Box& box) {
    // Squared point-box distance is quadratic between box-face crossings.
    const V d=b-a;std::vector<double> cuts{0,1};
    for(int k=0;k<3;++k) if(d[k]!=0) for(double face:{box.lo[k],box.hi[k]}) {
        const double t=(face-a[k])/d[k];if(t>0&&t<1) cuts.push_back(t);
    }
    std::sort(cuts.begin(),cuts.end());double best=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i+1<cuts.size();++i) {
        const double lo=cuts[i],hi=cuts[i+1],mid=(lo+hi)/2;double square=0,linear=0;
        best=std::min(best,std::min(PointBox(a+lo*d,box),PointBox(a+hi*d,box)));
        for(int k=0;k<3;++k) {
            const double p=a[k]+mid*d[k];
            if(p<box.lo[k]||p>box.hi[k]) {const double c=a[k]-(p<box.lo[k]?box.lo[k]:box.hi[k]);square+=d[k]*d[k];linear+=c*d[k];}
        }
        if(square>0) best=std::min(best,PointBox(a+std::max(lo,std::min(hi,-linear/square))*d,box));
    }
    return best;
}
Box WorldBox(const Box& box,const V& position,const Eigen::Matrix3d& rotation) {
    Box result{V::Constant(std::numeric_limits<double>::infinity()),V::Constant(-std::numeric_limits<double>::infinity()),box.id};
    for(unsigned i=0;i<8;++i) {const V p=position+rotation*V(i&1?box.hi.x():box.lo.x(),i&2?box.hi.y():box.lo.y(),i&4?box.hi.z():box.lo.z());result.lo=result.lo.cwiseMin(p);result.hi=result.hi.cwiseMax(p);}
    return result;
}
struct Collision {double minimum=std::numeric_limits<double>::infinity();std::size_t count=0;Json contacts=Json::array();};
Collision Collisions(const Spec& s,const CartesianPose& pose,const std::array<CableFreeSpan,8>& spans) {
    Collision out;const V position=Vec(pose.position_m);const auto rotation=Rotation(pose.orientation_xyzw);
    std::array<V,8> a,b;
    auto record=[&](const char* kind,const std::string& first,const std::string& second,double clearance) {
        Need(std::isfinite(clearance),"Nonfinite collision clearance.");out.minimum=std::min(out.minimum,clearance);
        if(clearance<=0) {
            ++out.count;
            // Endpoint mounting contacts must not hide a closer cable crossing.
            const auto at=std::find_if(out.contacts.begin(),out.contacts.end(),[&](const Json& c){return clearance<c.at("clearance_m").get<double>();});
            if(at!=out.contacts.end()||out.contacts.size()<4) {
                out.contacts.insert(at,Json{{"kind",kind},{"a",first},{"b",second},{"clearance_m",clearance}});
                if(out.contacts.size()>4)out.contacts.erase(out.contacts.end()-1);
            }
        }
    };
    for(int i=0;i<8;++i) {
        a[i]=Vec(spans[i].attachment_m);b[i]=Vec(spans[i].pulley_m);const double length=(b[i]-a[i]).norm();
        Need(length>2*s.exclusion+1e-10,"Attachment exclusions consume a complete free cable span.");
        const V direction=(b[i]-a[i])/length;a[i]+=direction*s.exclusion;b[i]-=direction*s.exclusion;
        const auto id="cable_"+std::to_string(i);
        if(s.body) for(const auto& body:s.bodies) record("cable_body",id,body.id,SegmentBox(rotation.transpose()*(a[i]-position),rotation.transpose()*(b[i]-position),body)-s.radius-s.clearance);
        for(const auto& box:s.obstacles) record("cable_obstacle",id,box.id,SegmentBox(a[i],b[i],box)-s.radius-s.clearance);
        for(int j=0;j<i;++j) record("cable_cable",id,"cable_"+std::to_string(j),SegmentDistance(a[i],b[i],a[j],b[j])-2*s.radius-s.clearance);
    }
    if(s.body&&(s.environment||s.bed_reference)) {
        for(const auto& component:s.bodies) {const auto body=WorldBox(component,position,rotation);
            for(const auto& box:s.obstacles) {const V gap=(body.lo-box.hi).cwiseMax(V::Zero()).cwiseMax(box.lo-body.hi);record("body_obstacle",component.id,box.id,gap.norm()-s.clearance);}}
    }
    return out;
}
struct Equilibrium {
    bool feasible=false;int rank=0;double force_residual=0,torque_residual=0;
    Eigen::Matrix<double,8,1> tensions=Eigen::Matrix<double,8,1>::Zero();
};
Equilibrium Statics(const Spec& s,const CartesianPose& pose,const std::array<CableFreeSpan,8>& spans) {
    const auto rotation=Rotation(pose.orientation_xyzw);const V origin=Vec(pose.position_m);
    Eigen::MatrixXd w(6,8);Eigen::VectorXd demand(6);
    const V gravity=s.mass*s.gravity;demand.head<3>()=-(gravity+s.force);demand.tail<3>()=-((rotation*s.cog).cross(gravity)+s.torque);
    for(int i=0;i<8;++i) {const V attachment=Vec(spans[i].attachment_m),delta=Vec(spans[i].pulley_m)-attachment;Need(delta.norm()>1e-12,"Degenerate free cable direction.");const V direction=delta.normalized();w.block<3,1>(0,i)=direction;w.block<3,1>(3,i)=(attachment-origin).cross(direction);}
    Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(w,Eigen::ComputeThinU|Eigen::ComputeThinV);decomposition.setThreshold(1e-10);
    Equilibrium out;out.rank=static_cast<int>(decomposition.rank());
    auto accept=[&](Eigen::Matrix<double,8,1> candidate) {
        if(!candidate.allFinite()) return false;
        for(int i=0;i<8;++i) {if(candidate[i]<s.lower[i]-1e-8||candidate[i]>s.upper[i]+1e-8) return false;candidate[i]=std::max(s.lower[i],std::min(s.upper[i],candidate[i]));}
        const Eigen::VectorXd residual=w*candidate-demand;
        if(residual.head<3>().norm()>1e-6||residual.tail<3>().norm()>1e-6) return false;
        out.feasible=true;out.tensions=candidate;out.force_residual=residual.head<3>().norm();out.torque_residual=residual.tail<3>().norm();return true;
    };
    const Eigen::Matrix<double,8,1> middle=s.upper_known?Eigen::Matrix<double,8,1>((s.lower+s.upper)/2):s.lower;
    if(accept(middle+decomposition.solve(demand-w*middle))) return out;
    // A small active-bound projection usually finds a feasible load witness
    // quickly. Failed projections fall through to complete vertex enumeration.
    std::vector<int> active;for(int i=0;i<8;++i)active.push_back(i);
    Eigen::Matrix<double,8,1> fixed_values=Eigen::Matrix<double,8,1>::Zero();
    while(!active.empty()) {
        Eigen::MatrixXd basis(6,static_cast<int>(active.size()));for(std::size_t j=0;j<active.size();++j)basis.col(static_cast<int>(j))=w.col(active[j]);
        Eigen::JacobiSVD<Eigen::MatrixXd> projected(basis,Eigen::ComputeThinU|Eigen::ComputeThinV);projected.setThreshold(1e-10);
        const Eigen::VectorXd solved=projected.solve(demand-w*fixed_values);Eigen::Matrix<double,8,1> candidate=fixed_values;
        int violation=-1;double excess=0,bound=0;
        for(std::size_t j=0;j<active.size();++j){const int index=active[j];candidate[index]=solved[static_cast<int>(j)];const double low=s.lower[index]-candidate[index],high=candidate[index]-s.upper[index];if(low>excess){excess=low;violation=static_cast<int>(j);bound=s.lower[index];}if(high>excess){excess=high;violation=static_cast<int>(j);bound=s.upper[index];}}
        if(accept(candidate))return out;
        if(violation<0)break;fixed_values[active[static_cast<std::size_t>(violation)]]=bound;active.erase(active.begin()+violation);
    }
    // Bounded equilibrium has a vertex if it is feasible. At a vertex at most
    // rank(W) variables are free; enumerate independent columns and fixed bounds.
    // Prefer vertices with fixed lower bounds before considering upper-bound
    // vertices, which can otherwise return an unnecessarily large allocation.
    for(unsigned pass=0;pass<(s.upper_cap?2u:1u);++pass) for(unsigned mask=0;mask<256;++mask) {
        std::vector<int> free,fixed;for(int i=0;i<8;++i) (mask&(1u<<i)?free:fixed).push_back(i);
        if(static_cast<int>(free.size())!=out.rank||free.empty()) continue;
        Eigen::MatrixXd basis(6,out.rank);for(int i=0;i<out.rank;++i) basis.col(i)=w.col(free[static_cast<std::size_t>(i)]);
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(basis);qr.setThreshold(1e-10);if(qr.rank()!=out.rank) continue;
        for(unsigned choice=pass?1u:0u;choice<(pass?(1u<<fixed.size()):1u);++choice) {
            Eigen::Matrix<double,8,1> candidate=middle;Eigen::VectorXd rhs=demand;
            for(std::size_t j=0;j<fixed.size();++j) {const int index=fixed[j];candidate[index]=choice&(1u<<j)?s.upper[index]:s.lower[index];rhs-=w.col(index)*candidate[index];}
            const Eigen::VectorXd solved=qr.solve(rhs);for(int i=0;i<out.rank;++i) candidate[free[static_cast<std::size_t>(i)]]=solved[i];
            if(accept(candidate)) return out;
        }
    }
    return out;
}
std::string BoundJob(const PrintJob& job,const RobotModelSnapshot& model) {
    const auto encoded=SerializePrintJob(job),frames=SerializeFrameRegistration(job.frames),actual=SerializeFrameRegistration(model.frames);
    Need(encoded.status==ResultStatus::success&&ValidateModelSnapshot(model).status==ResultStatus::success,"Invalid job/model for mechanics assessment.");
    Need(job.stage!=PlanningStage::imported&&job.model_identity.has_value,"Mechanics assessment requires a model-bound transformed or timed job.");
    const auto& a=job.model_identity.value;const auto& b=model.identity;
    Need(a.robot_config_sha256==b.robot_config_sha256&&a.calibration_sha256==b.calibration_sha256&&a.rail_state_sha256==b.rail_state_sha256&&a.motion_limits_sha256==b.motion_limits_sha256,"Mechanics model differs from job identity.");
    Need(frames.status==ResultStatus::success&&actual.status==ResultStatus::success&&frames.value==actual.value,"Mechanics registration differs from the job.");
    return Hash(encoded.value);
}
} // namespace

Result<std::string> AssessMechanics(const PrintJob& job,const RobotModelSnapshot& model,const std::string& physical_json) {
    Result<std::string> result;
    try {
        const auto job_hash=BoundJob(job,model);const auto spec=ReadSpec(Parse(physical_json));
        Need(spec.json.at("robot_config_sha256")==model.identity.robot_config_sha256,"Physical parameters belong to a different robot configuration.");
        std::size_t planned_samples=0;
        if(spec.collision||spec.statics) for(const auto& op:job.operations) if(op.motion.has_value) {
            Need(op.motion.value.end_effector.has_value,"Missing mechanics end-effector geometry.");const auto& g=op.motion.value.end_effector.value;
            const double distance=(Vec(g.end.position_m)-Vec(g.start.position_m)).norm();
            const double count=distance>1e-12?std::max(1.0,std::ceil(distance/spec.step))+1:1;
            Need(std::isfinite(count)&&count<=spec.per_motion,"Mechanics per-motion sample budget exceeded.");planned_samples+=static_cast<std::size_t>(count);
            Need(planned_samples<=spec.maximum,"Mechanics total sample budget exceeded; use a reviewed coarser interval or smaller job.");
        }
        Json samples=Json::array();std::size_t collision_poses=0,infeasible=0;int minimum_rank=6;double minimum_clearance=std::numeric_limits<double>::infinity();
        if(spec.collision||spec.statics) for(const auto& op:job.operations) if(op.motion.has_value) {
            const auto& g=op.motion.value.end_effector.value;
            Need(g.start.orientation_xyzw.x==g.end.orientation_xyzw.x&&g.start.orientation_xyzw.y==g.end.orientation_xyzw.y&&g.start.orientation_xyzw.z==g.end.orientation_xyzw.z&&g.start.orientation_xyzw.w==g.end.orientation_xyzw.w,"Mechanics v1 requires fixed orientation within each motion.");
            const double distance=(Vec(g.end.position_m)-Vec(g.start.position_m)).norm();const std::size_t intervals=distance>1e-12?static_cast<std::size_t>(std::max(1.0,std::ceil(distance/spec.step))):0;
            for(std::size_t i=0;i<=intervals;++i) {
                const double fraction=intervals?static_cast<double>(i)/static_cast<double>(intervals):0;CartesianPose pose=g.start;
                for(std::size_t k=0;k<3;++k) pose.position_m[k]=i==intervals&&intervals?g.end.position_m[k]:(1-fraction)*g.start.position_m[k]+fraction*g.end.position_m[k];
                const auto legacy=ToLegacyPose(pose);Need(legacy.status==ResultStatus::success,"Cannot convert mechanics pose.");const auto spans=ComputeCheckedCableSpans(model,legacy.value);Need(spans.status==ResultStatus::success,"Shared cable geometry rejected a mechanics sample.");
                Json sample={{"operation_id",op.id},{"progress",fraction},{"ee_world_m",pose.position_m},{"collision_count",nullptr},{"minimum_clearance_m",nullptr},{"contacts",Json::array()},
                    {"tension_status","not_evaluated"},{"tension_n",nullptr},{"breaking_strength_fraction",nullptr},{"wrench_rank",nullptr},{"force_residual_n",nullptr},{"torque_residual_nm",nullptr}};
                if(spec.collision) {const auto c=Collisions(spec,pose,spans.value);sample["collision_count"]=c.count;sample["minimum_clearance_m"]=c.minimum;sample["contacts"]=c.contacts;minimum_clearance=std::min(minimum_clearance,c.minimum);if(c.count) ++collision_poses;}
                if(spec.statics) {const auto t=Statics(spec,pose,spans.value);minimum_rank=std::min(minimum_rank,t.rank);sample["wrench_rank"]=t.rank;sample["tension_status"]=t.feasible?(spec.lower_known&&spec.upper_known?"feasible":"limits_unknown"):"infeasible";
                    if(t.feasible) {sample["tension_n"]=Json::array();if(spec.breaking_known)sample["breaking_strength_fraction"]=Json::array();for(int k=0;k<8;++k){sample["tension_n"].push_back(t.tensions[k]);if(spec.breaking_known)sample["breaking_strength_fraction"].push_back(t.tensions[k]/spec.breaking[k]);}sample["force_residual_n"]=t.force_residual;sample["torque_residual_nm"]=t.torque_residual;}else ++infeasible;}
                samples.push_back(std::move(sample));
            }
        }
        Json coverage=Json::array();if(spec.collision) {coverage.push_back("free_cable_pairs");if(spec.body) coverage.push_back("free_cables_vs_ee_components");if(spec.environment) coverage.push_back("free_cables_vs_declared_obstacles");if(spec.body&&spec.environment) coverage.push_back("ee_component_world_aabbs_vs_declared_obstacles");if(spec.bed_reference)coverage.push_back("bed_reference_footprint_plane");}
        Json report={{"schema_version",1},{"kind","cdpr_sampled_mechanics"},{"hardware_ready",false},{"job_id",job.job_id},{"job_sha256",job_hash},{"model_snapshot_sha256",model.snapshot_sha256},
            {"physical_model_sha256",Hash(spec.json.dump())},{"physical_source_sha256",Hash(physical_json)},{"physical_model",spec.json},{"sample_count",samples.size()},
            {"collision",{{"status",!spec.collision||samples.empty()?"not_evaluated":collision_poses?"potential_collision":"sampled_clear"},{"affected_samples",collision_poses},{"minimum_clearance_m",std::isfinite(minimum_clearance)?Json(minimum_clearance):Json(nullptr)},{"coverage",coverage}}},
            {"tension",{{"status",!spec.statics||samples.empty()?"not_evaluated":infeasible?"infeasible_at_samples":spec.lower_known&&spec.upper_known?"feasible_at_samples":"limits_not_evaluated"},{"infeasible_samples",infeasible},{"working_limits_known",spec.lower_known&&spec.upper_known},{"breaking_strength_known",spec.breaking_known},{"minimum_wrench_rank",spec.statics&&!samples.empty()?Json(minimum_rank):Json(nullptr)}}},
            {"limitations",Json::array({"Spatial samples only; intervals between samples are not certified.","Declared free-span capsules and conservative EE component box bounds only; excluded attachment zones, growing print, undeclared bed/frame hardware, sag and moving obstacles are not assessed.","Static equilibrium only: no inertia, friction, elasticity, feedback, thermal or hardware execution checks. A feasible allocation is not a measured or commanded cable force.","Breaking strength is an ultimate ceiling, not a working-load or pretension limit. Unknown working limits remain unevaluated.","Rank below six indicates incomplete wrench authority, even when the supplied static load balances."})},{"samples",std::move(samples)}};
        report["assessment_sha256"]=Hash(report.dump());result.value=report.dump(2);
        if(result.value.size()>24u*1024u*1024u)result.value=report.dump();
        Need(result.value.size()<=24u*1024u*1024u,"Mechanics report byte budget exceeded.");result.status=ResultStatus::success;
    } catch(const std::exception& error) {result.value.clear();result.status=ResultStatus::rejected;Diagnostic d;d.code=DiagnosticCode::invalid_configuration;d.message=error.what();result.diagnostics.push_back(d);}
    return result;
}

Result<bool> ValidateMechanicsAssessment(const PrintJob& job,const RobotModelSnapshot& model,const std::string& text) {
    Result<bool> result;
    try {
        auto root=Parse(text);Need(root.is_object()&&root.at("schema_version")==1&&root.at("kind")=="cdpr_sampled_mechanics"&&root.at("hardware_ready")==false,"Invalid mechanics report header.");
        const auto seal=root.at("assessment_sha256").get<std::string>();root.erase("assessment_sha256");Need(seal==Hash(root.dump()),"Mechanics report was changed after assessment.");
        Need(root.at("job_id")==job.job_id&&root.at("job_sha256")==BoundJob(job,model)&&root.at("model_snapshot_sha256")==model.snapshot_sha256,"Mechanics report belongs to a different job/model.");
        const auto spec=ReadSpec(root.at("physical_model"));Need(root.at("physical_model_sha256")==Hash(spec.json.dump())&&spec.json.at("robot_config_sha256")==model.identity.robot_config_sha256,"Mechanics physical model digest mismatch.");
        Need(root.at("samples").is_array()&&root.at("samples").size()<=spec.maximum&&root.at("sample_count")==root.at("samples").size(),"Invalid mechanics report sample count.");
        for(const auto& sample:root.at("samples")) {
            const auto id=sample.at("operation_id").get<std::size_t>();Need(id<job.operations.size()&&job.operations[id].id==id&&job.operations[id].motion.has_value,"Invalid mechanics sample operation.");
            const double u=Number(sample.at("progress"));Need(u>=0&&u<=1,"Invalid mechanics sample progress.");const V actual=Vec(sample.at("ee_world_m"));const auto& g=job.operations[id].motion.value.end_effector.value;
            Need((actual-((1-u)*Vec(g.start.position_m)+u*Vec(g.end.position_m))).norm()<1e-9,"Mechanics sample position differs from the job.");
        }
        result.status=ResultStatus::success;result.value=true;
    } catch(const std::exception& error) {result.status=ResultStatus::rejected;Diagnostic d;d.code=DiagnosticCode::identity_mismatch;d.message=error.what();result.diagnostics.push_back(d);}
    return result;
}
} // namespace cdpr_slicer
