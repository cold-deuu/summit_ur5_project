#include "rci_action_manager/simulation/simulation.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/fwd.hpp"
#include <iostream>
#include <cmath>

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace holistic_controller::math;

bool robot_ok_ = false;

int main(int argc, char ** argv){
    // ROS
    ros::init(argc, argv, "hqp_husky_franka");
    ros::NodeHandle nh;
    ros::Subscriber jointState = nh.subscribe("/mujoco/joint_states", 5, &jointStateCallback, ros::TransportHints().tcpNoDelay(true));
    summit_ur5_publisher_ = nh.advertise<std_msgs::Float64MultiArray>("/mujoco_ctrl",1);
    rviz_publisher_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // Robot Wrapper
    string model_path, urdf_name;
    // nh.getParam("/urdf_path", model_path);  
    // nh.getParam("/urdf_name",urdf_name);
    model_path = "/home/chan/catkin_ws/src/robot_description/summit_xl_description";
    urdf_name = "/summit_ur_control.urdf";

    string urdf_name2 = "/ur5e_control.urdf";


    vector<string> package_dirs;
    package_dirs.push_back(model_path);
    string urdfFileName = package_dirs[0] + urdf_name;
    robot_ = std::make_shared<holistic_controller::robot::RobotWrapper>(urdfFileName, package_dirs);
    ctrl_ = std::make_shared<RobotController::SummitUR5Wrapper>(robot_);
    ctrl_ ->initialize();



    // // Server
    se3_server_ = std::make_shared<SE3ActionServer>("rci_action_manager/se3Control", nh, ctrl_);
    joint_posture_server_ = std::make_shared<JointPostureActionServer>("rci_action_manager/jointControl", nh, ctrl_);

    Model model = robot_->model();
    Data data(model);
    int nq = model.nq;
    int nv = model.nv;
    int na = robot_->na();

    nq_ = nq;
    J_.resize(6,nv);
    q_.setZero(nq_);
    v_.setZero(nv);

    ros::Rate loop_rate(100);
  
    int i = 0;
    VectorXd qpub(9);
    qpub = q_;

    Reference desired;
  
    // desired.mobile_vel.resize(3);
    // desired.manip_pos.resize(nq_-3);

    desired.mobile_vel = Eigen::VectorXd::Zero(3);
    desired.manip_pos = q_.tail(nq-3);

    // bool controlFlag_ = false;

    while(ros::ok()){
        if (robot_ok_)
        {
          ctrl_->update_joint(q_, v_);
          se3_server_->compute(ros::Time::now());
          joint_posture_server_->compute(ros::Time::now());
          


          if(!se3_server_ ->isrunning() && !joint_posture_server_->isrunning())
          {
            desired = desired;
            desired.mobile_vel.setZero();
          }

          else
          {
            desired = ctrl_->get_result();
          }
          publish(desired);

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void publish(Reference des)
{
  Eigen::VectorXd des_vec = Eigen::VectorXd::Zero(nq_);
  des_vec.head(3) = des.mobile_vel;
  des_vec.tail(nq_-3) = des.manip_pos;

  // Mujoco
  std_msgs::Float64MultiArray msg;
  for(int i=0; i<des_vec.size(); i++) msg.data.push_back(des_vec(i));
  summit_ur5_publisher_.publish(msg);

  std::vector<std::string> joint_names = {
      "ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
      "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint",
      "gripper_finger1_joint"
  };

  // RVIZ
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names;
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.position.resize(joint_names.size());
  // joint_state_msg.position.assign(joint_names.size(), 0.0);  // 안전 초기화

  // std::cout<<q.size()<<std::endl;
  for(int i=0; i<des_vec.size()-3; i++) joint_state_msg.position[i] = des_vec(i+3);
  joint_state_msg.position.back() = 0.0;
  rviz_publisher_.publish(joint_state_msg);

}



void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  for(int i =0; i<9; i++)
  {
    q_(i) = msg->position[i];
    v_(i) = msg->velocity[i];
  }

  robot_ok_= true;

}

