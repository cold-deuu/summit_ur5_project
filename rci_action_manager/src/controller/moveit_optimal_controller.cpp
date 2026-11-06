#include "rci_action_manager/controller/moveit_optimal_controller.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>
#include <algorithm>
#include <limits>

namespace moveit_optimal {
pinocchio::Model armModel(const pinocchio::Model &model,const std::array<std::string,6> &names) {
    std::vector<pinocchio::JointIndex> locked;
    for(pinocchio::JointIndex i=1;i<model.joints.size();++i)
        if(std::find(names.begin(),names.end(),model.names[i])==names.end()) locked.push_back(i);
    if(locked.empty()) return model;
    // Retain all fixed geometry and inertias while locking passive wheels and
    // gripper at neutral. The realtime HQP still has exactly six arm variables.
    return pinocchio::buildReducedModel(model,locked,pinocchio::neutral(model));
}

const char *reasonText(Reason r) {
    switch(r) {
#define REASON(x,text) case Reason::x: return text;
    REASON(NONE,"") REASON(GOAL_REACHED,"goal tolerance satisfied")
    REASON(INVALID_NAMES,"joint_names must contain each configured joint exactly once")
    REASON(INVALID_POINTS,"invalid point dimensions or nonfinite values")
    REASON(INVALID_TIME,"need at least two points with strictly increasing nonnegative times")
    REASON(START_MISMATCH,"first point differs from current state beyond start_tolerance")
    REASON(STALE_STAMP,"header.stamp is in the past")
    REASON(BUSY,"trajectory staging buffers busy") REASON(REPLACED,"replaced by a newer trajectory")
    REASON(PATH_TOLERANCE,"path tolerance exceeded") REASON(GOAL_TOLERANCE,"goal tolerance timeout")
    REASON(STATE_TIMEOUT,"joint state unavailable or stale") REASON(FT_TIMEOUT,"F/T input unavailable or stale")
    REASON(QP_FAILURE,"HQP infeasible or numerical failure") REASON(JOINT_LIMIT,"measured joint limit exceeded")
    REASON(NOT_READY,"waiting for joint state and stationary F/T bias calibration")
    REASON(DRIVER_NOT_READY,"External Control inactive, scaling stale or speed zero")
    REASON(COMMAND_GAP,"latched fault: command-to-measured joint gap exceeded")
    REASON(INVALID_LIMITS,"trajectory exceeds configured position, velocity or acceleration bounds")
#undef REASON
    }
    return "unknown";
}
Segment Segment::make(double start,double duration,const Sample &s,const Sample &e) {
    Segment seg; seg.start=start; seg.duration=duration;
    const double T=duration;
    // Normalized-time quintic Hermite interpolation: shared q/v/a at knots => C2.
    const Vec c0=s.q,c1=s.v*T,c2=s.a*T*T/2;
    const Vec dp=e.q-c0-c1-c2, dv=e.v*T-c1-2*c2, da=e.a*T*T-2*c2;
    seg.coefficients.col(0)=c0; seg.coefficients.col(1)=c1; seg.coefficients.col(2)=c2;
    seg.coefficients.col(3)=10*dp-4*dv+.5*da;
    seg.coefficients.col(4)=-15*dp+7*dv-da;
    seg.coefficients.col(5)=6*dp-3*dv+.5*da;
    return seg;
}
Sample Segment::sample(double t) const {
    const double u=std::max(0.0,std::min(1.0,(t-start)/duration));
    Sample out;
    out.q=coefficients.col(5);
    for(int i=4;i>=0;--i) out.q=out.q*u+coefficients.col(i);
    out.v=5*coefficients.col(5);
    for(int i=4;i>=1;--i) out.v=out.v*u+i*coefficients.col(i);
    out.v/=duration;
    out.a=20*coefficients.col(5);
    for(int i=4;i>=2;--i) out.a=out.a*u+i*(i-1)*coefficients.col(i);
    out.a/=duration*duration;
    return out;
}
Sample Plan::sample(double elapsed) const {
    if(elapsed<=0) return first;
    if(elapsed>=duration) return last;
    size_t lo=0,hi=segments.size();
    while(lo+1<hi) {
        const size_t mid=(lo+hi)/2;
        if(segments[mid].start<=elapsed) lo=mid; else hi=mid;
    }
    return segments[lo].sample(elapsed);
}
bool Plan::buildPath() {
    path.clear();path.push_back({first.q,0,0});parameter_scale=0;
    // Sample the existing interpolant geometrically; timestamps are not a
    // schedule in this mode. Subdivision also resolves sparse long segments.
    for(const auto &segment:segments) {
        if(!segment.coefficients.allFinite())return false;
        double derivative_bound=0;
        for(int j=1;j<6;++j) derivative_bound+=j*segment.coefficients.col(j).norm();
        const int count=std::max(20,static_cast<int>(std::min(4096.0,std::ceil(derivative_bound/.005))));
        parameter_scale=std::max(parameter_scale,segment.sample(segment.start).v.norm());
        for(int j=1;j<=count;++j) {
            const double parameter=segment.start+segment.duration*j/count;
            const Sample value=segment.sample(parameter);const Vec q=value.q;
            parameter_scale=std::max(parameter_scale,value.v.norm());
            const double length=(q-path.back().q).norm();
            if(length>1e-10) {
                if(path.size()>=200000)return false;
                path.push_back({q,path.back().distance+length,parameter});
            }
        }
    }
    parameter_scale=std::max(1e-6,1.02*parameter_scale);
    return true; // Callers must observe an explicit successful path build.
}
double Plan::parameterAtDistance(double distance) const {
    if(path.size()<2 || distance>=path.back().distance)return duration*parameter_scale;
    size_t lo=0,hi=path.size()-1;
    while(lo+1<hi) {const size_t mid=(lo+hi)/2;if(path[mid].distance<=distance)lo=mid;else hi=mid;}
    const auto &a=path[lo];const auto &b=path[lo+1];
    const double fraction=std::max(0.0,std::min(1.0,(distance-a.distance)/(b.distance-a.distance)));
    return (a.parameter+fraction*(b.parameter-a.parameter))*parameter_scale;
}
Sample Plan::sampleProgress(double progress) const {
    Sample out=sample(progress/parameter_scale);
    out.v/=parameter_scale;out.a/=(parameter_scale*parameter_scale);
    return out;
}
Sample Plan::follow(const Vec &actual,double &progress,double lookahead,double window) const {
    Sample out;out.q=last.q;
    if(path.size()<2)return out;
    auto index=[&](double s) {
        size_t lo=0,hi=path.size()-1;
        while(lo+1<hi) {const size_t mid=(lo+hi)/2;if(path[mid].distance<=s)lo=mid;else hi=mid;}
        return lo;
    };
    const double begin=progress,end=std::min(path.back().distance,begin+window);
    double best=std::numeric_limits<double>::infinity(),nearest=begin;
    // Bound RT work and search only the next local section. Never search the
    // entire path: a later crossing must not capture progress.
    const size_t first_index=index(begin),stop=std::min(path.size()-1,first_index+64);
    for(size_t i=first_index;i<stop && path[i].distance<=end;++i) {
        const auto &a=path[i];const auto &b=path[i+1];
        const Vec delta=b.q-a.q;const double length=b.distance-a.distance;
        const double low=std::max(0.0,(begin-a.distance)/length);
        const double high=std::min(1.0,(end-a.distance)/length);
        const double fraction=std::max(low,std::min(high,delta.dot(actual-a.q)/delta.squaredNorm()));
        const double error=(actual-a.q-fraction*delta).squaredNorm();
        if(error<best) {best=error;nearest=a.distance+fraction*length;}
    }
    progress=std::max(progress,nearest);
    const double target=std::min(path.back().distance,progress+lookahead);
    out=sampleProgress(parameterAtDistance(target));out.v.setZero();out.a.setZero();
    // Projection only supplies a goal for the separate scalar governor.
    return out;
}
void PathProgressGovernor::reset() {
    position=velocity=acceleration=elapsed_=goal_=0;phase_=7;
    durations_.fill(0);jerks_.fill(0);
}
void PathProgressGovernor::startCruise(double goal,double max_v,double max_a,double max_j) {
    reset();goal_=std::max(0.0,goal);
    if(goal_<=1e-12)return;
    double tj=std::min(max_a/max_j,std::sqrt(max_v/max_j));
    double ta=std::max(0.0,max_v/(max_j*tj)-tj);
    double peak=max_j*tj*(tj+ta),cruise=0;
    const double ramp_distance=peak*(2*tj+ta);
    if(goal_>=ramp_distance)cruise=(goal_-ramp_distance)/peak;
    else if(goal_<2*max_a*max_a*max_a/(max_j*max_j)) {
        tj=std::cbrt(goal_/(2*max_j));ta=0;
    } else {
        tj=max_a/max_j;
        ta=std::max(0.0,(std::sqrt(tj*tj+4*goal_/max_a)-3*tj)/2);
    }
    durations_={tj,ta,tj,cruise,tj,ta,tj};
    jerks_={max_j,0,-max_j,0,-max_j,0,max_j};phase_=0;
}
void PathProgressGovernor::advance(double dt) {
    double remaining=dt;
    while(phase_<7 && remaining>0) {
        const double chunk=std::min(remaining,std::max(0.0,durations_[phase_]-elapsed_));
        const double jerk=jerks_[phase_];
        position+=velocity*chunk+acceleration*chunk*chunk/2+jerk*chunk*chunk*chunk/6;
        velocity+=acceleration*chunk+jerk*chunk*chunk/2;acceleration+=jerk*chunk;
        elapsed_+=chunk;remaining-=chunk;
        if(elapsed_>=durations_[phase_]-1e-12) {++phase_;elapsed_=0;}
    }
    if(phase_==7) {position=goal_;velocity=acceleration=0;}
}
double PathClock::advance(double target,double dt,double bandwidth) {
    target=std::max(0.0,std::min(1.0,target));
    const double error=rate-target,c=derivative+bandwidth*error;
    const double decay=std::exp(-bandwidth*dt);
    const double increment=target*dt+error*(1-decay)/bandwidth+
        c*(1-(1+bandwidth*dt)*decay)/(bandwidth*bandwidth);
    rate=target+(error+c*dt)*decay;
    derivative=(derivative-bandwidth*c*dt)*decay;
    return std::max(0.0,increment);
}
Controller::Controller(const Parameters &p)
 : hold_mode(p.hold_mode),tracking_admittance(p.tracking_admittance),p_(p),dt_(1/p.rate),
   robot_(std::make_shared<holistic_controller::robot::RobotWrapper>(p.urdf,std::vector<std::string>{})),
   model_(armModel(robot_->model(),p.joint_names)),actual_data_(model_),nominal_data_(model_),reference_data_(model_),task_("topic_joint_tracking")
{
    if(model_.nq!=6 || model_.nv!=6) throw std::invalid_argument("Topic controller requires a fixed-base six-joint model");
    for(int i=0;i<6;++i) {
        const auto id=model_.getJointId(p.joint_names[i]);
        if(id>=model_.joints.size() || model_.joints[id].idx_q()!=i)
            throw std::invalid_argument("Configured joint order differs from model order");
    }
    wrist_=model_.getJointId(p.wrist_joint);
    if(wrist_==0 || wrist_>=model_.joints.size())
        throw std::invalid_argument("Configured wrist_joint missing from arm model");
    hqp_.prepareJointTracking(p.regularization); task_.Kp(p.kp);
    admittance_.parameters=p.admittance;
    for(int i=0;i<3;++i) { slots_[i].store(0); plans_[i].segments.reserve(p.max_points); }
}
int Controller::acquirePlan() {
    for(int i=0;i<3;++i) { int free=0; if(slots_[i].compare_exchange_strong(free,1)) return i; }
    return -1;
}
void Controller::releasePlan(int slot) { slots_[slot].store(0); }
void Controller::submitPlan(int slot) {
    slots_[slot].store(2);
    const int previous=pending_.exchange(slot);
    if(previous>=0) slots_[previous].store(0);
}
void Controller::reject(Reason reason,uint64_t id) {
    rejected_id_.store(id); rejection_.store(static_cast<int>(reason));
}
void Controller::finish(Result result,Reason reason) {
    status_.result=result; status_.reason=reason; status_.result_id=status_.trajectory_id;
    status_.state=State::HOLDING;
    // Preserve the compliant reference on abort. Successful completion keeps the
    // trajectory endpoint as equilibrium; neither path resets admittance x/v.
    if(result==Result::ABORTED) hold_=command_-joint_offset_;
    if(active_>=0) slots_[active_].store(0);
    active_=-1; started_=false;
}
void Controller::timing(double ms,bool missed) {
    status_.loop_max_ms=std::max(status_.loop_max_ms,ms);
    if(missed) ++status_.missed_deadlines;
}
void Controller::step(double now) {
    joint_input.load(joints_); wrench_input.load(wrench_);
    const double wall_dt=last_step_wall_>0?std::max(0.0,std::min(.1,now-last_step_wall_)):0;
    last_step_wall_=now;
    status_.stamp=now; status_.sequence++;
    status_.bias_ready=wrench_.ready;
    status_.command_valid=false;
    if(fault_!=Reason::NONE) { status_.reason=fault_;output.store(status_);return; }
    if(!joints_.valid) { status_.reason=Reason::NOT_READY; output.store(status_); return; }
    const Vec q=Eigen::Map<const Vec>(joints_.q.data());
    const Vec v=Eigen::Map<const Vec>(joints_.v.data());
    if(!status_.initialized) {
        command_=hold_=nominal_.q=q; status_.initialized=true;
    }
    status_.actual=joints_.q;
    const bool fresh_state=now-joints_.stamp<p_.state_timeout;
    const bool fresh_ft=wrench_.ready && now-wrench_.stamp<p_.ft_timeout;
    if(!fresh_state || !fresh_ft) {
        if(active_>=0) finish(Result::ABORTED,fresh_state ? Reason::FT_TIMEOUT:Reason::STATE_TIMEOUT);
        status_.reason=wrench_.ready ? (fresh_state ? Reason::FT_TIMEOUT:Reason::STATE_TIMEOUT):Reason::NOT_READY;
        output.store(status_); return;
    }
    const double scale=p_.require_driver_ready?driver_scale.load():1.0;
    if(p_.require_driver_ready && (!driver_running.load() || scale<=0 ||
       now-driver_scaling_stamp.load()>p_.state_timeout)) {
        if(active_>=0) finish(Result::ABORTED,Reason::DRIVER_NOT_READY);
        status_.reason=Reason::DRIVER_NOT_READY;output.store(status_);return;
    }
    if((q.array()<p_.lower.array()-.01).any() || (q.array()>p_.upper.array()+.01).any()) {
        if(active_>=0) finish(Result::ABORTED,Reason::JOINT_LIMIT);
        status_.reason=Reason::JOINT_LIMIT;if(p_.latch_faults)fault_=Reason::JOINT_LIMIT; output.store(status_); return;
    }
    if(status_.result==Result::NONE) status_.reason=Reason::NONE;
    const int incoming=pending_.exchange(-1);
    if(incoming>=0) {
        if(active_>=0) finish(Result::ABORTED,Reason::REPLACED);
        // The new plan starts at the measured, already compliant posture.
        // Rebase accumulated displacement instead of adding the old HOLD
        // offset a second time. Preserve physical admittance velocity and
        // match the initial total command velocity through the C2 blend.
        if(p_.geometric_tracking) {
            // Preserve the preceding reference q/v/a for a C2 handover. The
            // new path's first point may differ from the command during lag.
            nominal_.q+=joint_offset_;nominal_.v+=offset_velocity_;
        } else {
            nominal_.q=command_;
            nominal_.v=previous_velocity_-((p_.admittance_enabled && tracking_admittance.load())?offset_velocity_:Vec::Zero());
            nominal_.a.setZero();
        }
        admittance_.displacement.setZero();
        admittance_.rotation.setIdentity(); // Keep log-rotation and displacement consistent.
        joint_offset_.setZero();
        active_=incoming; slots_[active_].store(3); started_=false;
        status_.trajectory_id=plans_[active_].id;
        start_=std::max(now,plans_[active_].start_time);plan_elapsed_=0;
        path_progress_=path_last_progress_=0;path_progress_stamp_=start_;
        path_reference_.reset();path_handover_elapsed_=0;
        path_clock_.reset();path_cruise_started_=false;
        status_.state=State::TRACKING;
        error_sum_=0; error_count_=0; status_.max_error=0; status_.rms=0;
    }
    const int rejected=rejection_.exchange(0);
    if(rejected) {
        status_.result=Result::REJECTED; status_.reason=static_cast<Reason>(rejected);
        status_.result_id=rejected_id_.load();
    }
    if(active_>=0 && now>=start_) {
        Plan &plan=plans_[active_];
        if(p_.geometric_tracking) {
            if(!started_) {
                Sample first;first.q=plan.first.q;
                blend_=Segment::make(0,p_.transition_duration,nominal_,first);started_=true;
            }
            if(path_handover_elapsed_<blend_.duration) {
                path_handover_elapsed_+=dt_*scale;
                nominal_=blend_.sample(path_handover_elapsed_);
                nominal_.v*=scale;nominal_.a*=scale*scale;
            } else {
                plan.follow(q,path_progress_,p_.path_lookahead,p_.path_search_window);
                if(!path_cruise_started_) {
                    path_reference_.startCruise(plan.duration*plan.parameter_scale,
                        p_.path_speed,p_.path_acceleration,p_.path_jerk);
                    path_cruise_started_=true;
                }
                // Do not stop at every projection/lookahead point. The measured
                // lag only slows a smooth path clock; the real endpoint is the
                // sole zero-velocity target of the S-curve.
                const double lag=(nominal_.q-q).norm();
                const double following=std::max(0.0,std::min(1.0,(2*p_.path_lookahead-lag)/p_.path_lookahead));
                const double increment=path_clock_.advance(scale*following,dt_,p_.path_clock_bandwidth);
                path_reference_.advance(increment);
                nominal_=plan.sampleProgress(path_reference_.position);
                const Vec derivative=nominal_.v;
                const double speed=path_reference_.velocity*path_clock_.rate;
                const double acceleration=path_reference_.acceleration*path_clock_.rate*path_clock_.rate+
                    path_reference_.velocity*path_clock_.derivative;
                nominal_.a=nominal_.a*speed*speed+derivative*acceleration;
                nominal_.v=derivative*speed;
            }
            if(path_progress_>path_last_progress_+1e-4) {
                path_last_progress_=path_progress_;path_progress_stamp_=now;
            }
            status_.elapsed=now-start_;
        } else {
        if(!started_) {
            // Replace only the initial portion, matching q/v/a at both ends.
            const double duration=std::min(p_.transition_duration,plan.duration);
            blend_=Segment::make(0,duration,nominal_,plan.sample(duration));
            started_=true;
        }
        // Advance the planner clock by the driver's scaling, while keeping
        // command publication and physical M-D-K integration at fixed 100 Hz.
        const double interval=std::min(wall_dt,std::max(0.0,now-start_));
        plan_elapsed_+=interval*scale;
        const double elapsed=plan_elapsed_;
        nominal_=elapsed<blend_.duration ? blend_.sample(elapsed):plan.sample(elapsed);
        nominal_.v*=scale;nominal_.a*=scale*scale;
        status_.elapsed=elapsed;
        }
    } else {
        nominal_.q=hold_; nominal_.v.setZero(); nominal_.a.setZero();
        status_.elapsed=active_>=0 ? now-start_:0;
    }

    pinocchio::forwardKinematics(model_,actual_data_,q,v);
    const Eigen::Matrix3d R=actual_data_.oMi[wrist_].rotation();
    Vec sensor=Eigen::Map<const Vec>(wrench_.value.data()),filtered,world;
    admittance_.filter(sensor,dt_,filtered);
    const Eigen::Vector3d force=p_.sensor_rotation*filtered.head<3>();
    const Eigen::Vector3d torque=p_.sensor_rotation*filtered.tail<3>()+p_.sensor_offset.cross(force);
    world.head<3>()=R*force; world.tail<3>()=R*torque;
    const bool tracking=active_>=0;
    admittance_.parameters.stiffness=tracking ? p_.tracking_stiffness :
        (hold_mode.load() ? Vec::Zero():p_.holding_stiffness);
    if(!p_.admittance_enabled || (tracking && !tracking_admittance.load())) {
        // Bypass the entire compliance state, not only the wrench input:
        // K=0 can otherwise retain HOLD displacement and residual velocity.
        // The start blend already matches the previous total command velocity.
        world.setZero();
        admittance_.displacement.setZero();admittance_.velocity.setZero();
        admittance_.rotation.setIdentity();
        joint_offset_.setZero();offset_velocity_.setZero();
    }
    admittance_.step(world,dt_);

    pinocchio::computeJointJacobians(model_,nominal_data_,nominal_.q);
    jacobian_.setZero();
    pinocchio::getJointJacobian(model_,nominal_data_,wrist_,pinocchio::LOCAL,jacobian_);
    const Eigen::Matrix3d Rn=nominal_data_.oMi[wrist_].rotation();
    Vec displacement,velocity;
    displacement.head<3>()=Rn.transpose()*admittance_.displacement.head<3>();
    displacement.tail<3>()=Rn.transpose()*admittance_.displacement.tail<3>();
    velocity.head<3>()=Rn.transpose()*admittance_.velocity.head<3>();
    velocity.tail<3>()=Rn.transpose()*admittance_.velocity.tail<3>();
    const Mat damped=jacobian_*jacobian_.transpose()+.0025*Mat::Identity();
    const Eigen::LDLT<Mat> inverse(damped);
    const Vec offset_target=jacobian_.transpose()*inverse.solve(displacement);
    Vec offset_velocity=jacobian_.transpose()*inverse.solve(velocity);
    if(p_.limit_joint_admittance) {
        const Vec requested=((offset_target-joint_offset_)/dt_)
            .cwiseMax(-p_.admittance_joint_velocity).cwiseMin(p_.admittance_joint_velocity);
        const double increment=p_.admittance_joint_acceleration*dt_;
        offset_velocity_+=(requested-offset_velocity_).cwiseMax(-increment).cwiseMin(increment);
        joint_offset_+=offset_velocity_*dt_;offset_velocity=offset_velocity_;
        // Spring mode must retain a fixed equilibrium; only free HOLD rebases
        // its reference to avoid accumulating lag behind measured joints.
        if(!tracking && p_.admittance_enabled && hold_mode.load()) {
            const Vec gap=(hold_+joint_offset_-q).cwiseMax(-p_.holding_reference_lag).cwiseMin(p_.holding_reference_lag);
            hold_=q+gap-joint_offset_;nominal_.q=hold_;
        }
    } else joint_offset_=offset_target;
    const Vec desired=nominal_.q+joint_offset_;
    Vec error=desired-q;
    const double path_error=error.cwiseAbs().maxCoeff();
    status_.nominal_error=(nominal_.q-q).cwiseAbs().maxCoeff();
    if(tracking) {
        error_sum_+=error.squaredNorm(); error_count_+=6;
        status_.rms=std::sqrt(error_sum_/error_count_);
        status_.max_error=std::max(status_.max_error,path_error);
        if(path_error>p_.path_tolerance) {
            finish(Result::ABORTED,Reason::PATH_TOLERANCE);
            nominal_.q=hold_; nominal_.v.setZero(); nominal_.a.setZero();
        } else if(started_ && (p_.geometric_tracking ?
                  (path_reference_.position>=plans_[active_].duration*plans_[active_].parameter_scale-1e-9 &&
                   std::abs(path_reference_.velocity)<1e-8) :
                  plan_elapsed_>=plans_[active_].duration)) {
            if(status_.nominal_error<p_.goal_tolerance &&
               v.cwiseAbs().maxCoeff()<p_.goal_velocity_tolerance) {
                hold_=plans_[active_].last.q;
                finish(Result::SUCCEEDED,Reason::GOAL_REACHED);
            } else if(p_.geometric_tracking ? now-path_progress_stamp_>p_.path_stall_timeout :
                      plan_elapsed_>plans_[active_].duration+p_.goal_timeout) {
                finish(Result::ABORTED,Reason::GOAL_TOLERANCE);
            }
        } else if(p_.geometric_tracking && now-path_progress_stamp_>p_.path_stall_timeout) {
            finish(Result::ABORTED,Reason::GOAL_TOLERANCE);
        }
    }
    Vec target=task_.velocityReference(nominal_.q+joint_offset_,
        nominal_.v+offset_velocity,nominal_.a,command_,dt_);
    Vec lower=-p_.velocity,upper=p_.velocity;
    if(p_.acceleration_limits) {
        lower=lower.cwiseMax(previous_velocity_-p_.acceleration*dt_);
        upper=upper.cwiseMin(previous_velocity_+p_.acceleration*dt_);
    }
    lower=lower.cwiseMax((p_.lower+Vec::Constant(p_.position_margin)-command_)/dt_);
    upper=upper.cwiseMin((p_.upper-Vec::Constant(p_.position_margin)-command_)/dt_);
    Vec nonlinear=Vec::Zero(),result;
    mass_.setIdentity();
    if(p_.torque_limits) {
        pinocchio::crba(model_,actual_data_,q);
        actual_data_.M.triangularView<Eigen::StrictlyLower>() =
            actual_data_.M.transpose().triangularView<Eigen::StrictlyLower>();
        mass_=actual_data_.M;
        nonlinear=pinocchio::nonLinearEffects(model_,actual_data_,q,v);
    }
    if(!hqp_.solveJointTracking(target,lower,upper,mass_,nonlinear,
                               previous_velocity_,p_.effort,dt_,p_.torque_limits,result)) {
        if(active_>=0) finish(Result::ABORTED,Reason::QP_FAILURE);
        if(p_.latch_faults)fault_=Reason::QP_FAILURE;
        status_.reason=Reason::QP_FAILURE;
        output.store(status_); return;
    }
    const Vec next=command_+result*dt_;
    if((next-q).cwiseAbs().maxCoeff()>p_.max_command_gap) {
        if(active_>=0)finish(Result::ABORTED,Reason::COMMAND_GAP);
        status_.reason=Reason::COMMAND_GAP;
        if(p_.latch_faults)fault_=Reason::COMMAND_GAP;
        output.store(status_);return;
    }
    command_=next; previous_velocity_=result;
    pinocchio::forwardKinematics(model_,reference_data_,nominal_.q+joint_offset_);
    auto pose=[](const pinocchio::SE3 &value,std::array<double,7> &out) {
        const Eigen::Quaterniond quaternion(value.rotation());
        for(int i=0;i<3;++i)out[i]=value.translation()(i);
        out[3]=quaternion.x();out[4]=quaternion.y();out[5]=quaternion.z();out[6]=quaternion.w();
    };
    pose(actual_data_.oMi[wrist_],status_.actual_pose);pose(nominal_data_.oMi[wrist_],status_.nominal_pose);
    pose(reference_data_.oMi[wrist_],status_.reference_pose);
    const Eigen::Vector3d translation=reference_data_.oMi[wrist_].translation()-actual_data_.oMi[wrist_].translation();
    const Eigen::Vector3d rotation=pinocchio::log3(Eigen::Matrix3d(reference_data_.oMi[wrist_].rotation()*R.transpose()));
    for(int i=0;i<3;++i) { status_.translation_error[i]=translation(i);status_.rotation_error[i]=rotation(i); }
    for(int i=0;i<6;++i) { status_.wrench_bias_corrected[i]=sensor(i);status_.wrench_filtered[i]=filtered(i); }
    status_.telemetry_valid=true;
    status_.command_valid=true;
    for(int i=0;i<6;++i) {
        status_.command[i]=command_(i); status_.velocity[i]=result(i);
        status_.desired[i]=nominal_.q(i)+joint_offset_(i); status_.error[i]=status_.desired[i]-q(i);
        status_.displacement[i]=admittance_.displacement(i);
        status_.admittance_velocity[i]=admittance_.velocity(i);
    }
    output.store(status_);
}
}
