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

//Task
// #include "cdpr_controllers/task/SE3_Track.hpp"
// #include "cdpr_controllers/task/Pf_ori_fix.hpp"
// #include "cdpr_controllers/task/Pf_pos_fix.hpp"
// #include "cdpr_controllers/task/Franka_pos_fix.hpp"


//CDPR Controller
// #include <cdpr/cdpr.h>
// #include <rci_cdpr_controller/trajectory/joint_cubic.hpp>
// #include <rci_cdpr_controller/trajectory/SE3_cubic.hpp>
// #include <rci_cdpr_controller/math/math.hpp>
// #include <rci_cdpr_controller/solver/HQP_solver.hpp>
// #include <rci_cdpr_controller/task/Joint_Posture_Task.hpp>

// RCI HOLISTIC CONTROLLER
#include <rci_holistic_controller/robot/robot.hpp>
#include <rci_holistic_controller/math/math.hpp>
#include <rci_holistic_controller/solver/HQP_solver.hpp>
#include <rci_holistic_controller/trajectory/SE3_cubic.hpp>
#include <rci_holistic_controller/trajectory/joint_cubic.hpp>


// SUMMIT UR5 SELF COLLISION AVOIDANCE
#include <sca_pkg/ur5e_sca_manager/ur5e_sca_manager.hpp>

//cpp header
#include <chrono>
#include <cmath>

using namespace std;
using namespace Eigen;
using namespace pinocchio;
using namespace holistic_controller::math;
using namespace holistic_controller::solver;




namespace RobotController{
    class SummitUR5Wrapper{
        public: 
            SummitUR5Wrapper(std::shared_ptr<holistic_controller::robot::RobotWrapper> robot);
            ~SummitUR5Wrapper(){};

            void initialize();
            void compute_all_terms();
            SE3 oMi();
            VectorXd get_result();
            void update_joint(Eigen::VectorXd q,  Eigen::VectorXd v);
            void update_ft(Eigen::VectorXd ft);

            // Control : SE3
            void initControl(SE3 target, ros::Duration duration, bool rel);
            void Control(ros::Time ctime);

            // Control : Joint
            void initJointPosture(VectorXd qTarget, ros::Duration duration);
            void jointPosture(ros::Time ctime);


            // Gravity Compensation
            VectorXd gravity_compensation();

            // Admittance Controller
            void initAdmittance();
            void AdmittanceControl();
            void AdmittanceControl_v2();
            Eigen::VectorXd compute_wrench();


        private:
            std::shared_ptr<holistic_controller::robot::RobotWrapper> robot_;
            std::shared_ptr<holistic_controller::solver::HQP_solver> solver_;
            std::shared_ptr<holistic_controller::trajectory::SE3CubicTrajectory> se3Ref_;
            std::shared_ptr<holistic_controller::trajectory::JointCubicTrajectory> jointRef_;
            std::shared_ptr<rci_summit_ur5_controller::SummitUR5_ScaManager> summitUr5_scaManager_;
            pinocchio::Model model_;
            pinocchio::Data data_;

            ros::Time stime_;
            ros::Duration duration_;

            VectorXd q_, v_, u_,m_;
            MatrixXd J_;

            int nq_, nv_, na_;

            SE3 oMi_;
            VectorXd q_des_;
            VectorXd desired_;

            // Admittance Controller            
            // FT Sensor
            VectorXd ft_sensor_; // Global
            MatrixXd Damping_;
            
            SE3 init_admittance_;
            VectorXd qdot_prev_;
            MatrixXd mass_;
            VectorXd nle_;

            // V2
            VectorXd des_twist_;
            MatrixXd M_a_, D_a_;
    


            
    };
}
#endif


