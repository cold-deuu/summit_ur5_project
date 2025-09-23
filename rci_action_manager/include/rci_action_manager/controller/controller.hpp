#ifndef __husky_franka_ctrl__
#define __husky_franka_ctrl__

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/QR>

// QP Header
#include "eiquadprog/eiquadprog.hpp"

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"


// RCI HOLISTIC CONTROLLER
#include <rci_holistic_controller/robot/robot.hpp>
#include <rci_holistic_controller/math/math.hpp>
#include <rci_holistic_controller/trajectory/SE3_cubic.hpp>
#include <rci_holistic_controller/trajectory/joint_cubic.hpp>

// RCI HOLISTIC CONTROLLER : TASK
#include <rci_holistic_controller/task/task_se3_equality.hpp>
#include <rci_holistic_controller/task/task_sca_inequality.hpp>
#include <rci_holistic_controller/task/task_base_fix_equality.hpp>
#include <rci_holistic_controller/task/task_joint_posture_equality.hpp>
#include <rci_holistic_controller/task/task_joint_velocity_limit_inequality.hpp>

// RCI HOLISTIC CONTROLLER : CONSTRAINT
#include <rci_holistic_controller/constraint/constraint_equality.hpp>
#include <rci_holistic_controller/constraint/constraint_inequality.hpp>

// RCI HOLISTIC CONTROLLER : HQP Data
#include <rci_holistic_controller/data/hqp_data.hpp>


// SUMMIT UR5 SELF COLLISION AVOIDANCE
#include <sca_pkg/ur5e_sca_manager/ur5e_sca_manager.hpp>

//cpp header
#include <chrono>
#include <cmath>

using namespace std;
using namespace Eigen;
using namespace pinocchio;
using namespace holistic_controller::math;




namespace RobotController{
    class SummitUR5Wrapper{
        public: 
            SummitUR5Wrapper(std::shared_ptr<holistic_controller::robot::RobotWrapper> robot, bool is_wholebody);
            ~SummitUR5Wrapper(){};

            void initialize();
            void compute_all_terms();
            SE3 oMi();
            Reference get_result();
            void update_joint(Eigen::VectorXd q,  Eigen::VectorXd v);

            // Control : SE3
            void initControl(SE3 target, ros::Duration duration, bool rel);
            void Control(ros::Time ctime);

            // Control : Joint
            void initJointPosture(VectorXd qTarget, ros::Duration duration);
            void jointPosture(ros::Time ctime);


            // Gravity Compensation
            VectorXd gravity_compensation();

            // State Ruturn
            State state()
            {
                return state_;
            }


        private:
            // RCI HQP Controller : Robot Dynamcis and Kinematics Solver
            std::shared_ptr<holistic_controller::robot::RobotWrapper> robot_;

            // RCI HQP Controller : Trajectory Manager
            std::shared_ptr<holistic_controller::trajectory::SE3CubicTrajectory> se3Ref_;
            std::shared_ptr<holistic_controller::trajectory::JointCubicTrajectory> jointRef_;

            // RCI HQP Controller : Self-Collision Mananger
            std::shared_ptr<rci_summit_ur5_controller::SummitUR5_ScaManager> summitUr5_scaManager_;
 
            // RCI HQP Controller : Task
            std::shared_ptr<chanhqp::task::TaskSE3Equality> se3Task_;
            std::shared_ptr<chanhqp::task::TaskJointPostureEquality> jointTask_;
            std::shared_ptr<chanhqp::task::TaskSCAInequality> scaTask_;
            std::shared_ptr<chanhqp::task::TaskBaseFixEquality> baseFixTask_;
            std::shared_ptr<chanhqp::task::TaskJointVelocityLimitInequality> jointVelLimitTask_;

            // RCI HQP Controller : Solver
            std::shared_ptr<chanhqp::data::HQPData> hqp_data_;


            // Pinocchio Params
            pinocchio::Model model_;
            pinocchio::Data data_;


            // ROS1 Params - Noetic
            ros::Time stime_;
            ros::Duration duration_;

            // Robot State
            State state_;

            // Control Input
            Reference ref_;
    
            // Joint Limit
            std::map<std::string, Eigen::VectorXd> joint_vel_limit_;

            // Option
            bool is_wholebody_;




            
    };
}
#endif


