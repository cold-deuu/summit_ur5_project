#include "rci_action_manager/controller/moveit_topic_node.hpp"
#include <ros/package.h>
#include <pinocchio/parsers/urdf.hpp>
#include <chrono>
#include <set>
#include <stdexcept>
#include <limits>
#include <std_msgs/Float64MultiArray.h>

namespace moveit_optimal {
namespace {
Vec vectorParam(ros::NodeHandle &nh,const std::string &name,const Vec &fallback,bool positive=false) {
    std::vector<double> values;
    if(!nh.getParam(name,values)) return fallback;
    if(values.size()!=6) throw std::invalid_argument(name+" must have six entries");
    Vec result=Eigen::Map<const Vec>(values.data());
    if(!result.allFinite() || (positive && (result.array()<=0).any()))
        throw std::invalid_argument("Invalid "+name);
    return result;
}
bool finite(const std::vector<double>& v) {
    return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});
}
}
Parameters TopicNode::loadParameters() {
    Parameters p;
    private_nh.param<std::string>("urdf",p.urdf,
        ros::package::getPath("summit_ur5e_moveit_config")+"/config/ur5e.urdf");
    std::vector<std::string> names;
    if(!private_nh.getParam("joint_names",names) || names.size()!=6 || std::set<std::string>(names.begin(),names.end()).size()!=6)
        throw std::invalid_argument("joint_names must list six unique model joints");
    std::copy(names.begin(),names.end(),p.joint_names.begin());
#define PARAM(name) private_nh.param(#name,p.name,p.name)
    PARAM(wrist_joint);
    PARAM(rate); PARAM(kp); PARAM(regularization); PARAM(start_tolerance); PARAM(path_tolerance);
    PARAM(goal_tolerance); PARAM(goal_velocity_tolerance); PARAM(goal_timeout);
    PARAM(transition_duration); PARAM(state_timeout); PARAM(ft_timeout); PARAM(position_margin);
    PARAM(torque_limits); PARAM(tracking_admittance);PARAM(admittance_enabled);PARAM(require_driver_ready);PARAM(limit_joint_admittance);PARAM(latch_faults);
    PARAM(admittance_joint_velocity);PARAM(admittance_joint_acceleration);PARAM(holding_reference_lag);PARAM(max_command_gap);
    PARAM(geometric_tracking);PARAM(admittance_aware_progress);PARAM(path_lookahead);PARAM(path_search_window);PARAM(path_stall_timeout);
    PARAM(path_speed);PARAM(path_acceleration);PARAM(path_jerk);
    PARAM(path_clock_bandwidth);
    PARAM(path_retiming);PARAM(retiming_velocity_scale);PARAM(retiming_acceleration_scale);PARAM(retiming_jerk_scale);PARAM(retiming_sample_period);
    PARAM(acceleration_limits);
    PARAM(servo_frame);PARAM(servo_gain);PARAM(servo_linear_speed);PARAM(servo_angular_speed);
    PARAM(servo_linear_acceleration);PARAM(servo_angular_acceleration);PARAM(servo_timeout);PARAM(servo_damping);PARAM(servo_max_travel);
#undef PARAM
    int max_points=5000; private_nh.param("max_points",max_points,max_points); p.max_points=max_points;
    std::string mode; private_nh.param<std::string>("admittance_mode",mode,"spring");
    if(mode!="spring" && mode!="hold") throw std::invalid_argument("admittance_mode must be spring or hold");
    p.hold_mode=mode=="hold";
    pinocchio::Model model; pinocchio::urdf::buildModel(p.urdf,model);
    model=armModel(model,p.joint_names);
    if(model.nq!=6 || model.nv!=6) throw std::invalid_argument("Arm reduction must have exactly six actuated joints");
    p.lower=model.lowerPositionLimit; p.upper=model.upperPositionLimit;
    p.velocity=vectorParam(private_nh,"limits/velocity",Vec::Constant(.4),true).cwiseMin(model.velocityLimit);
    p.acceleration=vectorParam(private_nh,"limits/acceleration",Vec::Constant(.8),true);
    p.jerk=vectorParam(private_nh,"limits/jerk",p.jerk,true);
    p.effort=vectorParam(private_nh,"limits/effort",Vec(model.effortLimit),true).cwiseMin(model.effortLimit);
    p.admittance.mass=vectorParam(private_nh,"admittance/mass",Vec::Constant(3),true);
    p.admittance.damping=vectorParam(private_nh,"admittance/damping",Vec::Constant(35),true);
    p.tracking_stiffness=vectorParam(private_nh,"admittance/tracking_stiffness",Vec::Constant(100));
    p.holding_stiffness=vectorParam(private_nh,"admittance/holding_stiffness",Vec::Constant(100));
    p.admittance.deadband=vectorParam(private_nh,"admittance/deadband",Vec::Constant(.8));
    p.admittance.displacement_limit=vectorParam(private_nh,"admittance/displacement_limit",Vec::Constant(.08),true);
    bool displacement_limits_enabled=true;
    private_nh.param("admittance/displacement_limits_enabled",displacement_limits_enabled,true);
    if(!displacement_limits_enabled)
        p.admittance.displacement_limit.setConstant(std::numeric_limits<double>::infinity());
    p.admittance.velocity_limit=vectorParam(private_nh,"admittance/velocity_limit",Vec::Constant(.06),true);
    p.admittance.acceleration_limit=vectorParam(private_nh,"admittance/acceleration_limit",p.admittance.acceleration_limit,true);
    private_nh.param("admittance/linear_velocity_norm_limit",p.admittance.linear_velocity_norm_limit,p.admittance.linear_velocity_norm_limit);
    private_nh.param("admittance/angular_velocity_norm_limit",p.admittance.angular_velocity_norm_limit,p.admittance.angular_velocity_norm_limit);
    private_nh.param("admittance/joint_velocity_limit",p.admittance_joint_velocity,p.admittance_joint_velocity);
    private_nh.param("admittance/joint_acceleration_limit",p.admittance_joint_acceleration,p.admittance_joint_acceleration);
    bool admittance_acceleration_limits_enabled=true;
    private_nh.param("admittance/acceleration_limits_enabled",admittance_acceleration_limits_enabled,true);
    if(!admittance_acceleration_limits_enabled) {
        p.admittance.acceleration_limit.setConstant(std::numeric_limits<double>::infinity());
        p.admittance_joint_acceleration=std::numeric_limits<double>::infinity();
    }
    private_nh.param("admittance/idle_reference_lag",p.holding_reference_lag,p.holding_reference_lag);
    private_nh.param("admittance/cutoff_hz",p.admittance.cutoff_hz,10.0);
    private_nh.param("admittance/wrench_sign",p.wrench_sign,-1.0);
    private_nh.param("admittance/bias_enabled",bias_enabled_,true);
    private_nh.param("admittance/bias_samples",bias_samples_,100);
    bias_=vectorParam(private_nh,"admittance/bias",Vec::Zero());
    calibrating_=bias_enabled_;
    std::vector<double> xyz,rpy;
    private_nh.param("admittance/sensor_to_wrist_xyz",xyz,std::vector<double>{0,0,.1});
    private_nh.param("admittance/sensor_to_wrist_rpy",rpy,std::vector<double>{0,0,0});
    if(xyz.size()!=3 || rpy.size()!=3 || !finite(xyz) || !finite(rpy))
        throw std::invalid_argument("Sensor transform requires finite xyz/rpy triples");
    p.sensor_offset=Eigen::Map<Eigen::Vector3d>(xyz.data());
    p.sensor_rotation=(Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX())).toRotationMatrix();
    if(!(p.rate>0 && p.rate<=1000 && p.kp>0 && p.regularization>0 &&
         p.start_tolerance>0 && p.path_tolerance>0 && p.goal_tolerance>0 &&
         p.goal_velocity_tolerance>0 && p.goal_timeout>0 && p.transition_duration>0 &&
         p.state_timeout>0 && p.ft_timeout>0 && p.position_margin>=0 &&
         p.admittance.cutoff_hz>0 && std::abs(p.wrench_sign)==1 &&
         bias_samples_>0 && max_points>=2) || (p.admittance.deadband.array()<0).any() || (p.tracking_stiffness.array()<0).any() ||
        (p.holding_stiffness.array()<0).any() || !(p.admittance.linear_velocity_norm_limit>0) ||
        !(p.admittance.angular_velocity_norm_limit>0) || !(p.admittance_joint_velocity>0) ||
        !(p.admittance_joint_acceleration>0) || !(p.holding_reference_lag>0) || !(p.max_command_gap>0) ||
        !std::isfinite(p.path_lookahead) || !(p.path_lookahead>0) ||
        !std::isfinite(p.path_search_window) || !(p.path_search_window>=p.path_lookahead) ||
        !std::isfinite(p.path_stall_timeout) || !(p.path_stall_timeout>0) ||
        !std::isfinite(p.path_speed) || !(p.path_speed>0) ||
        !std::isfinite(p.path_acceleration) || !(p.path_acceleration>0) ||
        !std::isfinite(p.path_jerk) || !(p.path_jerk>0) ||
        !std::isfinite(p.path_clock_bandwidth) || !(p.path_clock_bandwidth>0) ||
        !(p.retiming_velocity_scale>0 && p.retiming_velocity_scale<=1) ||
        !(p.retiming_acceleration_scale>0 && p.retiming_acceleration_scale<=1) ||
        !(p.retiming_jerk_scale>0 && p.retiming_jerk_scale<=1) ||
        !(p.retiming_sample_period>0 && std::isfinite(p.retiming_sample_period)) ||
        !(p.servo_gain>0 && p.servo_linear_speed>0 && p.servo_angular_speed>0 && p.servo_linear_acceleration>0 &&
          p.servo_angular_acceleration>0 && p.servo_timeout>0 && p.servo_damping>0 && p.servo_max_travel>0))
        throw std::invalid_argument("Invalid controller configuration");
    return p;
}
TopicNode::TopicNode() : private_nh("~") {
    params_=loadParameters();
    private_nh.param<std::string>("joint_state_topic",state_topic,"/mujoco/joint_states");
    private_nh.param<std::string>("command_topic",command_topic,"/mujoco_ctrl");
    private_nh.param<std::string>("wrench_topic",wrench_topic,"/mujoco/ft_sensor");
    private_nh.param<std::string>("wrench_frame",wrench_frame,"ee_site");
    private_nh.param("command_enabled",command_enabled_,false);
    controller_.reset(new Controller(params_));
    bool telemetry_enabled;private_nh.param("telemetry_enabled",telemetry_enabled,false);
    private_nh.param<std::string>("telemetry_frame",telemetry_frame_,"model_world");
    if(telemetry_enabled)telemetry_pub_=private_nh.advertise<rci_action_manager::ControlTelemetry>("telemetry",5);
    std::string topic; private_nh.param<std::string>("trajectory_topic",topic,"/rci_action_manager/moveit_plan");
    trajectory_sub_=nh.subscribe(topic,2,&TopicNode::receiveTrajectory,this);
    bool servo_enabled=false;private_nh.param("servo_enabled",servo_enabled,false);
    if(servo_enabled) {
        if(!controller_->servoAvailable()) throw std::invalid_argument("servo_enabled but servo_frame "+params_.servo_frame+" is not in the arm model");
        std::string servo_topic;private_nh.param<std::string>("servo_topic",servo_topic,"/rci_action_manager/servo_target");
        private_nh.param<std::string>("servo_reference_frame",servo_reference_frame_,"summit_xl_base_footprint");
        servo_sub_=nh.subscribe(servo_topic,5,&TopicNode::receiveServo,this);
        ROS_INFO_STREAM("[BRL_lab] servo enabled: "<<servo_topic<<" ("<<params_.servo_frame<<" in "<<servo_reference_frame_<<")");
    }
    status_pub_=private_nh.advertise<rci_action_manager::MoveitOptimalStatus>("status",1,true);
    command_timing_pub_=private_nh.advertise<std_msgs::Float64MultiArray>("command_timing",10);
    mode_service_=private_nh.advertiseService("set_admittance_mode",&TopicNode::setMode,this);
    bias_service_=private_nh.advertiseService("zero_ft",&TopicNode::zeroBias,this);
    enabled_service_=private_nh.advertiseService("set_tracking_admittance",&TopicNode::setEnabled,this);
    output_timer_=private_nh.createWallTimer(ros::WallDuration(1/params_.rate),&TopicNode::publish,this);
}
TopicNode::~TopicNode() {
    running_.store(false);
    if(control_thread_.joinable()) control_thread_.join();
}
bool TopicNode::snapshot(Snapshot &s) {
    for(int i=0;i<10;++i) if(controller_->output.load(s)) return true;
    return false;
}
void TopicNode::receiveState(const sensor_msgs::JointState::ConstPtr &msg) {
    JointState state;
    if(msg->position.size()!=msg->name.size() ||
       (!msg->velocity.empty() && msg->velocity.size()!=msg->name.size())) return;
    for(int i=0;i<6;++i) {
        const auto it=std::find(msg->name.begin(),msg->name.end(),params_.joint_names[i]);
        if(it==msg->name.end() || std::count(msg->name.begin(),msg->name.end(),*it)!=1) return;
        const size_t j=std::distance(msg->name.begin(),it);
        state.q[i]=msg->position[j]; state.v[i]=msg->velocity.empty()?0:msg->velocity[j];
        if(!std::isfinite(state.q[i]) || !std::isfinite(state.v[i])) return;
    }
    const double now=ros::Time::now().toSec();
    state.stamp=msg->header.stamp.isZero()?now:msg->header.stamp.toSec();
    if(state.stamp>now+.05) return;
    measured_velocity_.store(Eigen::Map<const Vec>(state.v.data()).cwiseAbs().maxCoeff());
    state.valid=true; controller_->joint_input.store(state);
}
void TopicNode::receiveWrench(const Vec &raw,double stamp,const std::string &frame) {
    if(!raw.allFinite() || (!frame.empty() && frame!=wrench_frame)) {
        ROS_WARN_THROTTLE(1,"Rejected nonfinite wrench or unexpected wrench frame"); return;
    }
    std::lock_guard<std::mutex> lock(bias_mutex_); // callback only, never control thread
    Wrench input;
    input.stamp=stamp;
    if(calibrating_) {
        Snapshot current;
        if(snapshot(current) && current.initialized &&
           measured_velocity_.load()<.01) {
            bias_sum_+=raw;
            if(++bias_count_>=bias_samples_) { bias_=bias_sum_/bias_count_; calibrating_=false; }
        }
    }
    const Vec corrected=params_.wrench_sign*(raw-bias_);
    for(int i=0;i<6;++i) input.value[i]=corrected(i);
    input.ready=!calibrating_;
    controller_->wrench_input.store(input);
}
void TopicNode::receiveServo(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if(!msg->header.frame_id.empty() && msg->header.frame_id!=servo_reference_frame_) {
        ROS_WARN_THROTTLE(1,"Rejected servo target in frame %s (expected %s)",msg->header.frame_id.c_str(),servo_reference_frame_.c_str());return;
    }
    const auto &p=msg->pose.position;const auto &o=msg->pose.orientation;
    const double norm=std::sqrt(o.x*o.x+o.y*o.y+o.z*o.z+o.w*o.w);
    if(!std::isfinite(p.x+p.y+p.z) || !std::isfinite(norm) || norm<.5) {ROS_WARN_THROTTLE(1,"Rejected nonfinite servo target");return;}
    const double now=ros::Time::now().toSec();
    Snapshot current;const bool known=snapshot(current);
    // New id when the stream was interrupted or the controller is not servoing this stream.
    if(last_servo_receive_<0 || now-last_servo_receive_>params_.servo_timeout ||
       (known && current.state!=State::SERVOING && current.result_id==servo_stream_id_)) servo_stream_id_=++next_id_;
    last_servo_receive_=now;
    ServoTarget target;target.stamp=now;target.id=servo_stream_id_;target.valid=true;
    target.pose={p.x,p.y,p.z,o.x/norm,o.y/norm,o.z/norm,o.w/norm};
    controller_->servo_input.store(target);
}
void TopicNode::receiveTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
    const uint64_t id=++next_id_;
    const auto reject=[&](Reason why) {
        controller_->reject(why,id);
        ROS_WARN_STREAM("Rejected trajectory "<<id<<": "<<reasonText(why));
    };
    Snapshot current;
    if(!snapshot(current) || !current.initialized || !current.bias_ready) { reject(Reason::NOT_READY); return; }
    if(msg->joint_names.size()!=6 ||
       std::set<std::string>(msg->joint_names.begin(),msg->joint_names.end()).size()!=6) { reject(Reason::INVALID_NAMES); return; }
    std::array<size_t,6> order;
    for(int i=0;i<6;++i) {
        auto it=std::find(msg->joint_names.begin(),msg->joint_names.end(),params_.joint_names[i]);
        if(it==msg->joint_names.end()) { reject(Reason::INVALID_NAMES); return; }
        order[i]=std::distance(msg->joint_names.begin(),it);
    }
    if(msg->points.size()<2 || msg->points.size()>params_.max_points) { reject(Reason::INVALID_TIME); return; }
    const double now=ros::Time::now().toSec();
    if(!msg->header.stamp.isZero() && msg->header.stamp.toSec()<now-.02) { reject(Reason::STALE_STAMP); return; }
    std::vector<Sample> samples(msg->points.size());
    std::vector<double> times(msg->points.size());
    bool have_vel=!msg->points.front().velocities.empty(),have_acc=!msg->points.front().accelerations.empty();
    for(size_t k=0;k<samples.size();++k) {
        const auto &point=msg->points[k];
        if(point.positions.size()!=6 || (have_vel?point.velocities.size()!=6:!point.velocities.empty()) ||
           (have_acc?point.accelerations.size()!=6:!point.accelerations.empty()) ||
           !finite(point.positions) || !finite(point.velocities) || !finite(point.accelerations) ||
           (!point.effort.empty() && (point.effort.size()!=6 || !finite(point.effort)))) {
            reject(Reason::INVALID_POINTS); return;
        }
        times[k]=point.time_from_start.toSec();
        if(times[k]<0 || (k && times[k]<=times[k-1])) { reject(Reason::INVALID_TIME); return; }
        for(int i=0;i<6;++i) {
            samples[k].q(i)=point.positions[order[i]];
            if(have_vel) samples[k].v(i)=point.velocities[order[i]];
            if(have_acc) samples[k].a(i)=point.accelerations[order[i]];
        }
    }
    if((samples.front().q-Eigen::Map<const Vec>(current.actual.data())).cwiseAbs().maxCoeff()>params_.start_tolerance) {
        reject(Reason::START_MISMATCH); return;
    }
    if(!have_vel) for(size_t k=1;k+1<samples.size();++k)
        samples[k].v=(samples[k+1].q-samples[k-1].q)/(times[k+1]-times[k-1]);
    if(!have_acc) for(size_t k=1;k+1<samples.size();++k)
        samples[k].a=(samples[k+1].v-samples[k-1].v)/(times[k+1]-times[k-1]);
    // A completed position trajectory must end at rest for HOLDING transfer.
    if(samples.back().v.cwiseAbs().maxCoeff()>.001) {
        reject(Reason::INVALID_POINTS); return;
    }
    samples.back().a.setZero(); // endpoint at rest, including acceleration
    const int slot=controller_->acquirePlan();
    if(slot<0) { reject(Reason::BUSY); return; }
    Plan &plan=controller_->plan(slot);
    plan.segments.clear();
    if(times.front()>0) {
        Sample first; first.q=Eigen::Map<const Vec>(current.actual.data());
        plan.segments.push_back(Segment::make(0,times.front(),first,samples.front()));
        plan.first=first;
    } else plan.first=samples.front();
    for(size_t k=1;k<samples.size();++k)
        plan.segments.push_back(Segment::make(times[k-1],times[k]-times[k-1],samples[k-1],samples[k]));
    for(const auto &segment:plan.segments) {
        // Validate the C2 interpolant too, not just the sparse waypoints.
        for(int j=0;j<=20;++j) {
            const Sample value=segment.sample(segment.start+segment.duration*j/20.0);
            if(!value.q.allFinite() || (value.q.array()<params_.lower.array()).any() ||
               (value.q.array()>params_.upper.array()).any() ||
               (!params_.geometric_tracking &&
                ((value.v.cwiseAbs().array()>params_.velocity.array()+.001).any() ||
                 (params_.acceleration_limits &&
                  (value.a.cwiseAbs().array()>params_.acceleration.array()+.001).any())))) {
                controller_->releasePlan(slot); reject(Reason::INVALID_LIMITS); return;
            }
        }
    }
    plan.last=samples.back(); plan.duration=times.back(); plan.timed=false;
    if(params_.geometric_tracking && params_.path_retiming) {
        std::string error;
        if(!retimePlan(plan,params_,error))
            ROS_WARN_STREAM("TOPP-RA retiming failed for trajectory "<<id<<", using path governor: "<<error);
    }
    if(params_.geometric_tracking && !plan.buildPath()) {
        ROS_WARN("Geometric path construction failed: nonfinite coefficients or path point budget exceeded");
        controller_->releasePlan(slot);reject(Reason::INVALID_POINTS);return;
    }
    plan.start_time=msg->header.stamp.isZero()?now:msg->header.stamp.toSec(); plan.id=id;
    controller_->submitPlan(slot);
}
bool TopicNode::setMode(rci_action_manager::SetAdmittanceMode::Request &req,rci_action_manager::SetAdmittanceMode::Response &res) {
    res.success=req.mode=="spring" || req.mode=="hold";
    if(res.success) controller_->hold_mode.store(req.mode=="hold");
    // Keep x and xdot: only K changes. HQP acceleration bounds keep commands continuous.
    res.message=res.success ? "Mode queued; displacement and velocity preserved":"mode must be spring or hold";
    return true;
}
bool TopicNode::setEnabled(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res) {
    controller_->tracking_admittance.store(req.data);
    res.success=true; res.message="Tracking admittance updated; existing offset retracts smoothly when disabled";
    return true;
}
bool TopicNode::zeroBias(std_srvs::Trigger::Request&,std_srvs::Trigger::Response &res) {
    Snapshot current;
    if(!snapshot(current) || current.state!=State::HOLDING ||
       Eigen::Map<const Vec>(current.velocity.data()).cwiseAbs().maxCoeff()>.01) {
        res.success=false; res.message="Bias calibration requires stationary HOLDING"; return true;
    }
    std::lock_guard<std::mutex> lock(bias_mutex_);
    calibrating_=true; bias_count_=0; bias_sum_.setZero();
    res.success=true; res.message="Collecting stationary unloaded F/T samples"; return true;
}
void TopicNode::publish(const ros::WallTimerEvent&) {
    Snapshot s;
    if(!snapshot(s)) return;
    if(command_enabled_ && s.command_valid && s.sequence!=last_command_sequence_) {
        const auto wall=std::chrono::steady_clock::now();
        const double stamp=ros::Time::now().toSec();
        const uint64_t gap=last_command_sequence_?s.sequence-last_command_sequence_:0;
        publish_command_(s);
        // Non-RT publication timing, not a robot/driver receipt timestamp.
        std_msgs::Float64MultiArray timing;
        timing.data={stamp,have_command_time_?std::chrono::duration<double,std::milli>(wall-last_command_wall_).count():0,
            1000*(stamp-s.stamp),static_cast<double>(gap)};
        command_timing_pub_.publish(timing);
        last_command_wall_=wall;have_command_time_=true;last_command_sequence_=s.sequence;
    }
    rci_action_manager::MoveitOptimalStatus msg;
    msg.header.stamp=ros::Time(s.stamp);
    msg.state=s.state==State::TRACKING?"TRACKING":(s.state==State::SERVOING?"SERVOING":"HOLDING");
    static const char *results[]={"NONE","SUCCEEDED","REJECTED","ABORTED"};
    msg.last_result=results[static_cast<int>(s.result)]; msg.reason=reasonText(s.reason);
    msg.trajectory_id=s.trajectory_id; msg.result_trajectory_id=s.result_id;
    msg.command_enabled=command_enabled_; msg.ft_bias_ready=s.bias_ready;
    msg.elapsed=s.elapsed; msg.tracking_rms=s.rms; msg.tracking_max=s.max_error;
    msg.nominal_error_max=s.nominal_error;
    msg.loop_duration_max_ms=s.loop_max_ms; msg.missed_deadlines=s.missed_deadlines;
    for(int i=0;i<6;++i) {
        msg.actual[i]=s.actual[i]; msg.desired[i]=s.desired[i]; msg.command[i]=s.command[i];
        msg.command_velocity[i]=s.velocity[i]; msg.tracking_error[i]=s.error[i];
        msg.admittance_displacement[i]=s.displacement[i]; msg.admittance_velocity[i]=s.admittance_velocity[i];
    }
    status_pub_.publish(msg);
    if(telemetry_pub_ && s.telemetry_valid) {
        rci_action_manager::ControlTelemetry data;data.header=msg.header;data.header.frame_id=telemetry_frame_;
        data.mode=s.state==State::TRACKING?"JOINT_TRAJECTORY":(s.state==State::SERVOING?"SERVO":"HOLDING");data.action_id=s.trajectory_id;
        data.reason=(s.reason==Reason::GOAL_REACHED || s.reason==Reason::REPLACED)?"":reasonText(s.reason);data.command_enabled=command_enabled_;data.ft_bias_ready=s.bias_ready;
        data.fault_latched=s.reason==Reason::COMMAND_GAP || s.reason==Reason::QP_FAILURE || s.reason==Reason::JOINT_LIMIT;
        auto pose=[](const std::array<double,7> &value,geometry_msgs::Pose &out) {
            out.position.x=value[0];out.position.y=value[1];out.position.z=value[2];
            out.orientation.x=value[3];out.orientation.y=value[4];out.orientation.z=value[5];out.orientation.w=value[6];
        };
        pose(s.actual_pose,data.actual_pose);pose(s.nominal_pose,data.nominal_pose);pose(s.reference_pose,data.reference_pose);
        for(int i=0;i<3;++i) { data.translation_error[i]=s.translation_error[i];data.rotation_error[i]=s.rotation_error[i]; }
        for(int i=0;i<6;++i) {
            data.joint_actual[i]=s.actual[i];data.joint_command[i]=s.command[i];data.joint_error[i]=s.command[i]-s.actual[i];
            data.wrench_bias_corrected[i]=s.wrench_bias_corrected[i];data.wrench_filtered[i]=s.wrench_filtered[i];
            data.admittance_displacement[i]=s.displacement[i];data.admittance_velocity[i]=s.admittance_velocity[i];
        }
        telemetry_pub_.publish(data);
    }

}
void TopicNode::run(const std::function<void(const Snapshot&)> &publisher) {
    publish_command_=publisher; running_.store(true);
    control_thread_=std::thread([this]() {
        using Clock=std::chrono::steady_clock;
        const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1/params_.rate));
        auto next=Clock::now();
        while(running_.load() && ros::ok()) {
            const auto begin=Clock::now();
            controller_->step(ros::Time::now().toSec());
            const auto end=Clock::now(); next+=period;
            controller_->timing(std::chrono::duration<double,std::milli>(end-begin).count(),end>next);
            if(end>next+period) next=end;
            std::this_thread::sleep_until(next); // the sole intentional wait in the control thread
        }
    });
    ros::AsyncSpinner spinner(2); spinner.start();
    ros::waitForShutdown(); running_.store(false); control_thread_.join();
}
}
