//Holistic Controller
//#include "cdpr_controllers/controller/controller.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/compute-all-terms.hxx"
#include <Eigen/QR>    
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "eiquadprog/eiquadprog.hpp"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <tuple>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include <gazebo_msgs/LinkState.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>

#include "rci_holistic_controller/robot/robot.hpp"
#include "rci_holistic_controller/math/math.hpp"
#include "rci_action_manager/controller/controller.hpp"
#include "rci_action_manager/server/wholebody_server.hpp"
#include "rci_action_manager/server/joint_posture_server.hpp"


//SYSTEM Header
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>

//HuskyFrankaAction
// #include <cdpr_controllers/server/wholebody_server.hpp>
// #include <cdpr_controllers/server/joint_posture_server.hpp>
// #include <cdpr_controllers/server/platform_se3_server.hpp>
// #include <cdpr_controllers/server/franka_se3_server.hpp>
// #include <cdpr/cdpr.h>

using namespace std;
using namespace Eigen;

// Robot Wrapper
std::shared_ptr<holistic_controller::robot::RobotWrapper> robot_;
std::shared_ptr<RobotController::SummitUR5Wrapper> ctrl_;
std::shared_ptr<SE3ActionServer> se3_server_;
std::shared_ptr<JointPostureActionServer> joint_posture_server_;

//varialbes - const
int nq_;


//Subs
// ros::Publisher armPub1_, armPub2_, armPub3_, armPub4_, armPub5_, armPub6_, mobilePub_;
ros::Publisher summit_ur5_publisher_;
//msg
std::vector<ros::Publisher> pubSet_;

//Action Server
// std::unique_ptr<SE3ActionServer> wholebody_action_server_;
// std::unique_ptr<JointPostureActionServer> joint_posture_action_server_;
// std::unique_ptr<PlatformSE3ActionServer> platform_action_server_;
// std::unique_ptr<FrankaSE3ActionServer> franka_action_server_;


//globla_variable
Eigen::VectorXd q_,v_,l_;     
Eigen::VectorXd base_q_, base_v_;
Eigen::MatrixXd J_;

// Sensor
Eigen::VectorXd ft_sensor_;
// pinocchio::SE3 wTep_;
// double dur_;

std::vector<std::string> jointOrder_;

//Callback
void get_joint_state();
void gazebo_publish_(std::vector<ros::Publisher> u, Eigen::VectorXd q);
void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void FTSensorCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);


// Admittance
bool admit_flag_, init_admit_;