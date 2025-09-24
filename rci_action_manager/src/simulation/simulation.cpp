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
bool is_wholebody_;
int main(int argc, char ** argv){
    // ROS
    ros::init(argc, argv, "hqp_summit_ur");
    ros::NodeHandle nh;

    // ROS : Subscriber
    ros::Subscriber jointState = nh.subscribe("/mujoco/joint_states", 5, &jointStateCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber ftSensor = nh.subscribe("/mujoco/ft_sensor", 5, &FTSensorCallback, ros::TransportHints().tcpNoDelay(true));

    // ROS : Publisher
    summit_ur5_publisher_ = nh.advertise<std_msgs::Float64MultiArray>("/mujoco_ctrl",1);
    rviz_publisher_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

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
    robot_ = std::make_shared<holistic_controller::robot::RobotWrapper>(urdfFileName, package_dirs);
    ctrl_ = std::make_shared<RobotController::SummitUR5Wrapper>(robot_, is_wholebody_);
    ctrl_ ->initialize();

    // Server
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
    ft_sensor_.setZero(6);

    ros::Rate loop_rate(100);

    Reference desired;
    desired.mobile_vel = Eigen::VectorXd::Zero(3);
    desired.manip_pos = Eigen::VectorXd::Zero(6);
    // desired.manip_pos = q_.tail(nq-3);
    desired.manip_pos << 0.0, -1.57, 1.57, -1.57, -1.57, 0.0;


    while(ros::ok()){
        if (robot_ok_)
        {
          ctrl_->update_joint(q_, v_);
          ctrl_->update_ft(ft_sensor_);
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
        
        // std::cout<<"[Simulator] q : \n"<<q_.transpose()<<std::endl;

    }

}

void publish(Reference des)
{
  Eigen::VectorXd des_vec = Eigen::VectorXd::Zero(nq_);
  // Whole body
  if (is_wholebody_)
  {
    des_vec.head(3) = des.mobile_vel;
    des_vec.tail(nq_-3) = des.manip_pos;
  }
  // Manip Only
  else
  {
    des_vec.tail(nq_) = des.manip_pos;
  }

  // Mujoco
  std_msgs::Float64MultiArray msg;
  for(int i=0; i<des_vec.size(); i++) msg.data.push_back(des_vec(i));
  summit_ur5_publisher_.publish(msg);



  // RVIZ
  std::vector<std::string> joint_names = {
      "ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
      "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint",
      "gripper_finger1_joint"
  };

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name = joint_names;
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.position.resize(joint_names.size());

  // std::cout<<q.size()<<std::endl;
  if (is_wholebody_)
  {
    for(int i=0; i<des_vec.size()-3; i++) joint_state_msg.position[i] = des_vec(i+3);
  }
  else
  {
    for(int i=0; i<des_vec.size(); i++) joint_state_msg.position[i] = des_vec(i);
  }
  joint_state_msg.position.back() = 0.0;
  rviz_publisher_.publish(joint_state_msg);

}



void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  
  for(int i =0; i<nq_; i++)
  {
    if (is_wholebody_)
    {
      q_(i) = msg->position[i];
      v_(i) = msg->velocity[i];
    }
    else
    {
      q_(i) = msg->position[i+3];
      v_(i) = msg->velocity[i+3];
    }
  }

  robot_ok_= true;

}

void FTSensorCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  for (int i=0; i<6; i++)
    ft_sensor_(i) = msg->data[i];  

      // std::copy(msg->data.begin() + 6, msg->data.end(), ft_sensor_.data());
}