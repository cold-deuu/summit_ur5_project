#ifndef __husky_franka_ctrl__
#define __husky_franka_ctrl__

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

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
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"


// RCI HOLISTIC CONTROLLER
#include <rci_holistic_controller/robot/robot.hpp>
#include <rci_holistic_controller/math/math.hpp>
#include <rci_holistic_controller/trajectory/SE3_cubic.hpp>
#include <rci_holistic_controller/trajectory/joint_cubic.hpp>
#include <rci_holistic_controller/trajectory/joint_cubic_array.hpp>

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
#include <rci_holistic_controller/admittance/cartesian_admittance.hpp>
#include <rci_action_manager/ControlTelemetry.h>
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
            bool telemetry(rci_action_manager::ControlTelemetry &out) const;
            void set_control_ready(bool ready) { control_ready_=ready; }
            bool control_ready() const { return control_ready_; }
            void set_speed_scaling(double scale) { speed_scaling_=std::max(0.0,std::min(1.0,scale)); }
            double speed_scaling() const { return speed_scaling_; }
            uint64_t generation() const { return generation_; }
            const pinocchio::Model &model() const { return model_; }
            Eigen::VectorXd joint_velocity_limits() const {
                return joint_vel_limit_.at("upper").tail(state_.n_manip);
            }
            bool joint_settled(double tolerance) const {
                return (state_.q.tail(state_.n_manip)-joint_goal_-joint_admittance_offset_).cwiseAbs().maxCoeff()<tolerance &&
                       state_.v.tail(state_.n_manip).cwiseAbs().maxCoeff()<.02;
            }
            bool se3_settled(double position_tolerance,double rotation_tolerance) const;
            void update_joint(Eigen::VectorXd q,  Eigen::VectorXd v);

            // Control : SE3
            void initControl(SE3 target, ros::Duration duration, bool rel);
            void Control(ros::Time ctime);

            // Control : Joint
            void initJointPosture(VectorXd qTarget, ros::Duration duration);
            void jointPosture(ros::Time ctime);

            // Control : Moveit
            void initMoveitPlanner(trajectory_msgs::JointTrajectory msg);
            void ControlMoveitPlanner();
            int moveit_idx_, moveit_target_idx_;
            bool moveit_succed_, moveit_abort_;

            // Control : Admittance
            void update_ft(Eigen::VectorXd ft);
            void update_admittance_control();

            // Gravity Compensation
            VectorXd gravity_compensation();

            // State Ruturn
            State state()
            {
                return state_;
            }




        private:
            holistic_controller::admittance::CartesianAdmittance action_admittance_, idle_admittance_;
            bool admittance_enabled_=true;
            void reset_action_admittance();
            Eigen::Matrix<double, 6, 1> update_action_admittance();
            Eigen::Matrix<double, 6, 1> action_displacement_, action_velocity_;
            Eigen::Matrix3d action_rotation_;
            Eigen::VectorXd joint_admittance_offset_;
            Eigen::Vector3d sensor_offset_;
            Eigen::Matrix3d sensor_rotation_ = Eigen::Matrix3d::Identity();
            bool idle_active_ = false, control_ready_ = true;
            double speed_scaling_=1.0, wrench_sign_=-1.0;
            uint64_t generation_=0;
            std::string wrist_joint_="ur5e_wrist_3_joint";
            Eigen::VectorXd joint_goal_;
            SE3 se3_goal_, nominal_pose_=SE3::Identity(), reference_pose_=SE3::Identity();
            bool telemetry_valid_=false, smooth_wrench_=false;
            std::string control_mode_="HOLDING";
            holistic_controller::admittance::CartesianAdmittance wrench_filter_;
            Eigen::Matrix<double,6,1> wrench_unfiltered_=Eigen::Matrix<double,6,1>::Zero();
            double admittance_joint_velocity_=3, admittance_joint_acceleration_=1e9, idle_reference_lag_=1e9;
            Eigen::VectorXd idle_qdot_, joint_admittance_qdot_;
            SE3 joint_pose(const Eigen::VectorXd &arm_q);
            // RCI HQP Controller : Robot Dynamcis and Kinematics Solver
            std::shared_ptr<holistic_controller::robot::RobotWrapper> robot_;

            // RCI HQP Controller : Trajectory Manager
            std::shared_ptr<holistic_controller::trajectory::SE3CubicTrajectory> se3Ref_;
            std::shared_ptr<holistic_controller::trajectory::JointCubicTrajectory> jointRef_;
            std::shared_ptr<holistic_controller::trajectory::JointCubicTrajectoryArray> jointArrayRef_;

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

            // Admittance Controller
            Eigen::VectorXd ft_sensor_, xdot_admit_;
            
            // Control Input
            Reference ref_;
    
            // Joint Limit
            std::map<std::string, Eigen::VectorXd> joint_vel_limit_;

            // Option
            bool is_wholebody_;

            // for Control
            Data data_ctrl_;
            

            // Control : Moveit
            std::vector<Eigen::VectorXd> moveit_traj_;
            
            
            Eigen::Matrix<double, 6, 6> M_adm_;
            Eigen::Matrix<double, 6, 6> D_adm_;
            Eigen::Matrix<double, 6, 6> K_adm_;

            Eigen::Matrix<double, 6, 1> x_adm_;     
            Eigen::Matrix<double, 6, 1> xdot_adm_;  
            Eigen::Matrix<double, 6, 1> xddot_adm_; 


            double dt_ = 0.01;

            
    };
}
#endif


