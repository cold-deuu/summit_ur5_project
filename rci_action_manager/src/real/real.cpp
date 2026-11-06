#include "rci_action_manager/real/real.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/fwd.hpp"
#include <iostream>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rci_action_manager/MoveitOptimalStatus.h>
#include <rci_action_manager/controller/position_command_output.hpp>

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace holistic_controller::math;

bool robot_ok_ = false;
bool is_wholebody_;
bool arm_ready_ = false, base_ready_ = false;
ros::Publisher mobile_publisher_;
std::unique_ptr<rci_hardware::PositionCommandOutput> position_output_;
std::vector<std::string> arm_joint_names_;
bool command_enabled_=false, publish_command_states_=false, require_driver_ready_=false;
bool driver_program_running_=false, ft_ready_=false;
bool admittance_enabled_=true;
int bias_samples_=0, bias_count_=0;
Eigen::Matrix<double,6,1> ft_bias_=Eigen::Matrix<double,6,1>::Zero(), ft_bias_sum_=Eigen::Matrix<double,6,1>::Zero();
ros::Time arm_stamp_, ft_stamp_, base_stamp_, scaling_stamp_, program_stamp_;
double speed_scaling_=0, state_timeout_=.25, max_position_step_=.01, command_horizon_=.01;
std::string wrench_frame_, safety_reason_, fault_reason_;
void storeFT(const Eigen::Matrix<double,6,1>& value,ros::Time stamp) {
    if (!value.allFinite()) return;
    if(stamp>ros::Time::now()+ros::Duration(.05)) {
        ROS_WARN_THROTTLE(2,"F/T stamp is ahead of local time; synchronize driver-host and HQP-host clocks");return;
    }
    ft_stamp_=stamp;
    if (!ft_ready_) {
        if (arm_ready_ && v_.tail(6).cwiseAbs().maxCoeff()<.01) {
            ft_bias_sum_+=value;
            if (++bias_count_>=bias_samples_) { ft_bias_=ft_bias_sum_/bias_count_; ft_ready_=true; }
        } else { bias_count_=0; ft_bias_sum_.setZero(); }
    }
    ft_sensor_=value-ft_bias_;
}
void stampedFTCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    if (msg->header.frame_id!=wrench_frame_ || msg->header.stamp.isZero()) return;
    const auto &w=msg->wrench;
    Eigen::Matrix<double,6,1> value;
    value<<w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z;
    storeFT(value,msg->header.stamp);
}
void baseStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!is_wholebody_ || q_.size() != 9) return;
    const auto& p = msg->pose.pose;
    const auto& t = msg->twist.twist;
    const double yaw = tf::getYaw(p.orientation);
    if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(yaw) || !std::isfinite(t.linear.x) ||
        !std::isfinite(t.linear.y) || !std::isfinite(t.angular.z)) return;
    q_.head(3) << p.position.x, p.position.y, yaw;
    // Odometry twist is child-local; virtual x/y coordinates are world-frame.
    v_.head(3) << cos(yaw)*t.linear.x-sin(yaw)*t.linear.y,
                  sin(yaw)*t.linear.x+cos(yaw)*t.linear.y, t.angular.z;
    base_stamp_=msg->header.stamp.isZero()?ros::Time::now():msg->header.stamp;
    base_ready_ = true;
    robot_ok_ = arm_ready_;
}
int main(int argc, char ** argv){
    // ROS
    ros::init(argc, argv, "hqp_summit_ur_real");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    string odom_topic, base_command_topic, state_topic, command_topic;
    pnh.param("command_enabled",command_enabled_,false);
    pnh.param("admittance_enabled",admittance_enabled_,true);
    pnh.param("publish_command_joint_states",publish_command_states_,false);
    pnh.param("require_driver_ready",require_driver_ready_,false);
    pnh.param("state_timeout",state_timeout_,.25);
    pnh.param("max_position_step",max_position_step_,.01);
    pnh.param("command_horizon",command_horizon_,.01);
    pnh.param("ft_bias_samples",bias_samples_,0);
    ft_ready_=bias_samples_==0;
    if (!(state_timeout_>0 && max_position_step_>0 && command_horizon_>=.01 && bias_samples_>=0))
        throw std::invalid_argument("Invalid real interface settings");
    pnh.param<string>("joint_state_topic",state_topic,"/brl_robot/ur5e/joint_states");
    pnh.param<string>("command_topic",command_topic,"/brl_robot/ur5e/ur5e_scaled_pos_joint_traj_controller/command");
    std::string telemetry_frame;
    pnh.param<std::string>("telemetry_frame",telemetry_frame,"model_world");
    ros::Publisher telemetry_pub=pnh.advertise<rci_action_manager::ControlTelemetry>("telemetry",5);
    ros::Publisher status_pub=pnh.advertise<rci_action_manager::MoveitOptimalStatus>("status",1);
    std::string scaling_topic,program_topic;
    pnh.param<std::string>("speed_scaling_topic",scaling_topic,"/speed_scaling_factor");
    pnh.param<std::string>("driver_program_topic",program_topic,"/ur_hardware_interface/robot_program_running");
    ros::Subscriber scaling=nh.subscribe<std_msgs::Float64>(scaling_topic,1,
      +[](const std_msgs::Float64::ConstPtr &m) { if (std::isfinite(m->data) && m->data>=0 && m->data<=1) {
          speed_scaling_=m->data; scaling_stamp_=ros::Time::now(); } });
    ros::Subscriber program=nh.subscribe<std_msgs::Bool>(program_topic,1,
      +[](const std_msgs::Bool::ConstPtr &m) { driver_program_running_=m->data; program_stamp_=ros::Time::now(); });
    pnh.param<string>("base_state_topic", odom_topic, "/brl_robot/odom");
    pnh.param<string>("base_command_topic", base_command_topic, "/brl_robot/cmd_vel");
    ros::Subscriber baseState;
    if(nh.param("/whole_body",false)) {
        baseState=nh.subscribe(odom_topic,5,&baseStateCallback);
        mobile_publisher_=nh.advertise<geometry_msgs::Twist>(base_command_topic,1);
    }

    // ROS : Subscriber
    ros::Subscriber jointState = nh.subscribe(state_topic, 5, &jointStateCallback, ros::TransportHints().tcpNoDelay(true));
    string ft_topic;
    pnh.param<string>("ft_topic", ft_topic, "/mujoco/ft_sensor");
    bool stamped_ft=false; pnh.param("stamped_ft",stamped_ft,false);
    pnh.param<string>("wrench_frame",wrench_frame_,"wrist_3_link");
    ros::Subscriber ftSensor;
    if (stamped_ft) ftSensor=nh.subscribe(ft_topic,5,&stampedFTCallback,ros::TransportHints().tcpNoDelay(true));
    else ftSensor=nh.subscribe(ft_topic,5,&FTSensorCallback,ros::TransportHints().tcpNoDelay(true));

    // ROS : Publisher
    // summit_ur5_publisher_ = nh.advertise<std_msgs::Float64MultiArray>("/brl_robot/ur5e/ur5e_scale_pos_joint_traj_controller/command",1);
    // Position transport is initialized below after model joint order is known.
    rviz_publisher_ = nh.advertise<sensor_msgs::JointState>("/hqp_action/command_joint_states", 1);

    // Robot Wrapper
    string model_path, urdf_name;

    model_path = ros::package::getPath("summit_xl_description");
    nh.getParam("/whole_body", is_wholebody_);  
    if (is_wholebody_)
    {
      ROS_INFO_STREAM("Wholebody Controller");
      urdf_name = "/summit_ur_wholebody_control.urdf";
    }
    else
    {
      ROS_INFO_STREAM("Manipulator Controller");
      urdf_name = "/summit_ur_manip_control.urdf";
    } 

    vector<string> package_dirs;
    package_dirs.push_back(model_path);
    string urdfFileName = package_dirs[0] + urdf_name;
    pnh.param<string>("model_file", urdfFileName, urdfFileName);
    robot_ = std::make_shared<holistic_controller::robot::RobotWrapper>(urdfFileName, package_dirs);
    ctrl_ = std::make_shared<RobotController::SummitUR5Wrapper>(robot_, is_wholebody_);
    ctrl_ ->initialize();
    ctrl_->set_control_ready(false);

    // Server
    se3_server_ = std::make_shared<SE3ActionServer>("rci_action_manager/se3Control", nh, ctrl_);
    joint_posture_server_ = std::make_shared<JointPostureActionServer>("rci_action_manager/jointControl", nh, ctrl_);
    bool enable_moveit_action; pnh.param("enable_moveit_action",enable_moveit_action,true);
    if (enable_moveit_action) moveit_planner_server_ = std::make_shared<MoveitActionServer>("rci_action_manager/moveitControl", nh, ctrl_);



    Model model = robot_->model();
    Data data(model);
    int nq = model.nq;
    int nv = model.nv;
    int na = robot_->na();
    int n_mobile, n_manip;
    if(is_wholebody_) 
      n_mobile = 3;
    else n_mobile = 0;
    n_manip = nq - n_mobile;

    if (n_manip!=6 || (is_wholebody_ && nq!=9)) throw std::invalid_argument("Expected arm6 or base3+arm6 model");
    arm_joint_names_.resize(6);
    for (pinocchio::JointIndex j=1;j<model.joints.size();++j) {
        int index=model.joints[j].idx_q()-n_mobile;
        if (index>=0 && index<6) arm_joint_names_[index]=model.names[j];
    }
    position_output_.reset(new rci_hardware::PositionCommandOutput(nh,pnh,arm_joint_names_,command_enabled_,command_topic));
    nq_ = nq;
    J_.resize(6,nv);
    q_.setZero(nq_);
    v_.setZero(nv);
    ft_sensor_.setZero(6);

    ros::Rate loop_rate(100);

    Reference desired;
    desired.mobile_vel = Eigen::VectorXd::Zero(3);
    desired.manip_pos = Eigen::VectorXd::Zero(6);
    // desired.manip_pos = q_.tail(nq-3);
    // desired.manip_pos << 0.0, -1.57, 1.57, -1.57, -1.57, 0.0;



    // Moveit Planning Mode 로 Switching 을 해야할 듯?
    // 아니면 Melodic 에서 Client 요청을 보내도 되긴 하는데 그냥 Servoing 모드로 바꾸는게 맞는 것 같음
    if (ros::ok())
    {
      std::cout<<"ROS IS READY"<<std::endl;
    }
    else std::cout<<"ROS IS NOT READY"<<std::endl;

    while(ros::ok()){
      ros::spinOnce();
      const ros::Time now=ros::Time::now();
      std::string reason;
      if (!fault_reason_.empty()) reason=fault_reason_;
      else if (!robot_ok_ || (now-arm_stamp_).toSec()>state_timeout_ ||
          (is_wholebody_ && (now-base_stamp_).toSec()>state_timeout_)) reason="joint/base state unavailable or stale";
      // F/T is a required control input only when compliance is enabled.
      // Continue bias collection/logging while nominal-only tracking runs.
      else if (admittance_enabled_ && (!ft_ready_ || (now-ft_stamp_).toSec()>state_timeout_)) reason="waiting for fresh F/T and stationary bias";
      else if (require_driver_ready_ && (!driver_program_running_ ||
               (now-scaling_stamp_).toSec()>state_timeout_ || speed_scaling_<=0)) reason="External Control inactive, scaling stale or speed zero";
      const bool ready=reason.empty();
      ctrl_->set_control_ready(ready);
      ctrl_->set_speed_scaling(require_driver_ready_?speed_scaling_:1.0);
      if (ready)
      {
        ctrl_->update_joint(q_, v_);
        


        ctrl_->update_ft(ft_sensor_);
        //   ctrl_->update_ft(ft_sensor_);
        se3_server_->compute(ros::Time::now());
        joint_posture_server_->compute(ros::Time::now());
        if (moveit_planner_server_) moveit_planner_server_->compute(now);

        if(!se3_server_ ->isrunning() && !joint_posture_server_->isrunning() && (!moveit_planner_server_ || !moveit_planner_server_->isrunning()))
        {
          // std::cout<<"[Real] server : \n"<<q_.transpose()<<std::endl;
          ctrl_->update_admittance_control();
          desired = ctrl_->get_result();

          desired_bf_.manip_pos = desired.manip_pos;
        }

        else
        {
          desired = ctrl_->get_result();
        }
        bool safety = check_safety(desired);
        
        if (safety) desired_bf_ = desired;
        // publish(desired_bf_);
        if (!safety) {
            fault_reason_="latched fault: "+safety_reason_;
            reason=fault_reason_;
            ROS_ERROR_STREAM(reason << "; no further commands until node restart");
            ctrl_->set_control_ready(false);
            if (se3_server_->isrunning()) se3_server_->signalAbort(true);
            if (joint_posture_server_->isrunning()) joint_posture_server_->signalAbort(true);
        } else if (command_enabled_) publish(desired_bf_);

      }
      else {
        if(se3_server_->isrunning() || joint_posture_server_->isrunning() ||
           (moveit_planner_server_ && moveit_planner_server_->isrunning()))
            ROS_WARN_STREAM("[Real] Stopping active action: "<<reason);
        if (se3_server_->isrunning()) se3_server_->signalAbort(true);
        if (joint_posture_server_->isrunning()) joint_posture_server_->signalAbort(true);
        if (moveit_planner_server_ && moveit_planner_server_->isrunning()) moveit_planner_server_->signalAbort(true);
        ROS_INFO_THROTTLE(2.0, "[Real] Waiting for fresh state/F/T/driver readiness");
      }
      rci_action_manager::MoveitOptimalStatus status;
      status.header.stamp=now; status.command_enabled=command_enabled_; status.ft_bias_ready=ft_ready_;
      status.last_result=fault_reason_.empty()?"NONE":"ABORTED";
      status.state=(se3_server_->isrunning() || joint_posture_server_->isrunning())?"TRACKING":"HOLDING";
      status.reason=reason;
      if (arm_ready_) for (int i=0;i<6;++i) {
          status.actual[i]=q_(n_mobile+i);
          status.desired[i]=desired.manip_pos(i); status.command[i]=desired.manip_pos(i);
          status.tracking_error[i]=desired.manip_pos(i)-q_(n_mobile+i);
      }
      for(int i=0;i<6;++i) status.nominal_error_max=std::max(status.nominal_error_max,std::abs(status.tracking_error[i]));
      status_pub.publish(status);
      rci_action_manager::ControlTelemetry telemetry;
      if(ctrl_->telemetry(telemetry)) {
          telemetry.header.stamp=now;telemetry.header.frame_id=telemetry_frame;
          telemetry.command_enabled=command_enabled_;telemetry.ft_bias_ready=ft_ready_;
          telemetry.fault_latched=!fault_reason_.empty();telemetry.reason=reason;
          telemetry_pub.publish(telemetry);
      }

      ros::spinOnce();
      loop_rate.sleep();
      
      // std::cout<<"[Simulator] q : \n"<<q_.transpose()<<std::endl;
      // std::cout<<"[Simulator] v : \n"<<v_.transpose()<<std::endl;
      
  }

}

bool check_safety(Reference desired)
{
  safety_reason_.clear();
  if (desired.manip_pos.size()!=6 || (is_wholebody_ && desired.mobile_vel.size()!=3)) {
    safety_reason_="invalid command dimensions"; return false;
  }
  if (!desired.manip_pos.allFinite()) { safety_reason_="nonfinite joint command"; return false; }
  // Joint Vel
  for(int i=0; i<6; i++)
  {
    if(abs(q_(q_.size()-6+i) - desired.manip_pos(i)) >max_position_step_)
    {
      safety_reason_="joint command step exceeded: "+arm_joint_names_[i]+
        " actual="+std::to_string(q_(q_.size()-6+i))+" command="+std::to_string(desired.manip_pos(i))+
        " limit="+std::to_string(max_position_step_);
      return false;
    }
  }

  // Joint Limit
  for(int i=0; i<6; i++)
  {
    if(desired.manip_pos(i) < robot_->model().lowerPositionLimit(q_.size()-6+i) ||
       desired.manip_pos(i) > robot_->model().upperPositionLimit(q_.size()-6+i))
    {
      safety_reason_="joint position limit exceeded: "+arm_joint_names_[i];
      return false;
    }
  }


  if (!desired.mobile_vel.allFinite()) { safety_reason_="nonfinite base command"; return false; }
  if (is_wholebody_ && (desired.mobile_vel.head(2).cwiseAbs().maxCoeff() > .2 ||
                        abs(desired.mobile_vel(2)) > .3)) { safety_reason_="base velocity limit exceeded"; return false; }
  return true;
}


void publish(Reference des)
{
  const Eigen::VectorXd& des_vec = des.manip_pos;
  if (is_wholebody_) {
    geometry_msgs::Twist twist;
    const double yaw = q_(2);
    twist.linear.x = cos(yaw)*des.mobile_vel(0)+sin(yaw)*des.mobile_vel(1);
    twist.linear.y = -sin(yaw)*des.mobile_vel(0)+cos(yaw)*des.mobile_vel(1);
    twist.angular.z = des.mobile_vel(2);
    mobile_publisher_.publish(twist);
  }

  // RVIZ
  const auto &joint_names=arm_joint_names_;


  // Real
  // std_msgs::Float64MultiArray msg;
  position_output_->publish(des_vec.data());



  if (!publish_command_states_) return;
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names;
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.position.resize(joint_names.size());

  // std::cout<<"[Simulator] Des vec"<<des_vec.transpose()<<std::endl;
  for(int i=0; i<des_vec.size(); i++) joint_state_msg.position[i] = des_vec(i);
  rviz_publisher_.publish(joint_state_msg);

}



void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  const auto &joint_names=arm_joint_names_;

  const int offset = is_wholebody_ ? 3 : 0;
  if (q_.size() != offset+6) return;
  Eigen::Matrix<double,6,1> positions, velocities;
  for (size_t i=0;i<joint_names.size();++i) {
      auto it=std::find(msg->name.begin(),msg->name.end(),joint_names[i]);
      if (it==msg->name.end()) return;
      size_t index=std::distance(msg->name.begin(),it);
      if (index>=msg->position.size() || index>=msg->velocity.size() ||
          !std::isfinite(msg->position[index]) || !std::isfinite(msg->velocity[index])) return;
      positions(i)=msg->position[index]; velocities(i)=msg->velocity[index];
  }
  const ros::Time stamp=msg->header.stamp.isZero()?ros::Time::now():msg->header.stamp;
  if (stamp>ros::Time::now()+ros::Duration(.05)) {
      ROS_WARN_THROTTLE(2,"Joint state stamp is %.3f s ahead; synchronize driver-host and HQP-host clocks",
                        (stamp-ros::Time::now()).toSec());return;
  }
  q_.tail(6)=positions; v_.tail(6)=velocities;
  arm_stamp_=stamp;
  arm_ready_=true;
  robot_ok_=!is_wholebody_ || base_ready_;
}

void FTSensorCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size()!=6) return;
  for (int i=0; i<6; i++) {
    if (!std::isfinite(msg->data[i])) return;
  }
  storeFT(Eigen::Map<const Eigen::Matrix<double,6,1>>(msg->data.data()),ros::Time::now());  

      // std::copy(msg->data.begin() + 6, msg->data.end(), ft_sensor_.data());
}
