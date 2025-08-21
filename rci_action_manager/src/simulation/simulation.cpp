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
    ros::Subscriber ftSensor = nh.subscribe("/mujoco/ft_sensor", 5, &FTSensorCallback, ros::TransportHints().tcpNoDelay(true));
    
    summit_ur5_publisher_ = nh.advertise<std_msgs::Float64MultiArray>("/mujoco_ctrl",1);
    
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



    // Server
    se3_server_ = std::make_shared<SE3ActionServer>("rci_action_manager/se3Control", nh, ctrl_);
    joint_posture_server_ = std::make_shared<JointPostureActionServer>("rci_action_manager/jointControl", nh, ctrl_);

    Model model = robot_->model();
    Data data(model);
    int nq = model.nq;
    int nv = model.nv;
    int na = robot_->na();


    J_.resize(6,nv);
    ft_sensor_.resize(6);
    q_.setZero(9);
    v_.setZero(9);


    ros::Rate loop_rate(100);
    

        
    int i = 0;
    VectorXd qpub(9);
    qpub = q_;
    bool controlFlag_ = false;

    admit_flag_ = false;
    init_admit_=  true;

    while(ros::ok()){
        if (robot_ok_)
        {
          ctrl_->update_joint(q_, v_);
          ctrl_->update_ft(ft_sensor_);
          // std::cout<<"Force Torque Sensor :"<<ft_sensor_.transpose()<<std::endl;
          se3_server_->compute(ros::Time::now());
          joint_posture_server_->compute(ros::Time::now());
          


          if(!se3_server_ ->isrunning() && !joint_posture_server_->isrunning())
          {
            if(admit_flag_)
            {            
              if (init_admit_)
              {
                ctrl_->initAdmittance();
                init_admit_ = false;
              }
              else
              {
                ctrl_->AdmittanceControl();
                // ctrl_->AdmittanceControl_v2();

              }
              qpub = ctrl_->get_result();
              // std::cout<<"---------Admittance Control-----------"<<std::endl;
            }
            else{
              qpub = qpub;
            }

          }
            // qpub = qpub;
          else
          {
            admit_flag_ = true;
            qpub = ctrl_->get_result();
          }
          

          // std::cout<<"Q Pub :"<<qpub.transpose()<<std::endl;

          std_msgs::Float64MultiArray msg;
          for(int i=0; i<qpub.size(); i++) msg.data.push_back(qpub(i));
          summit_ur5_publisher_.publish(msg);
          
          // std::cout<<"oMi : "<<ctrl_->oMi()<<std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}




void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  for(int i =0; i<9; i++)
  {
    q_(i) = msg->position[i];
    v_(i) = msg->velocity[i];
  }

  robot_ok_= true;

}

void FTSensorCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  for (int i=0; i<6; i++)
    ft_sensor_(i) = msg->data[i];  

      // std::copy(msg->data.begin() + 6, msg->data.end(), ft_sensor_.data());
}