#include "rci_action_manager/controller/controller.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace std;

using namespace holistic_controller::math;

namespace RobotController
{
    SummitUR5Wrapper::SummitUR5Wrapper(std::shared_ptr<holistic_controller::robot::RobotWrapper> robot, bool is_wholebody)
    : robot_(robot), is_wholebody_(is_wholebody)
    {
        model_ = robot_->model();
        Data data(model_);
        data_ = data;


        // na_ = robot_->na();

        // is wholebody 추가
        state_.n_var = model_.nv;
        if (is_wholebody_) state_.n_mobile = 3;
        else state_.n_mobile = 0;
        state_.n_manip = model_.nv -state_.n_mobile;

        ref_.mobile_vel.resize(state_.n_mobile);
        ref_.manip_pos.resize(state_.n_manip);

        // Trajectory
        se3Ref_ = std::make_shared<holistic_controller::trajectory::SE3CubicTrajectory>("SE3_Trajectory");
        jointRef_ = std::make_shared<holistic_controller::trajectory::JointCubicTrajectory>("SE3_Trajectory", state_.n_manip);
        
        // Self-Collisoin Library Loading
        summitUr5_scaManager_ = std::make_shared<rci_summit_ur5_controller::SummitUR5_ScaManager>();

        // Task
        se3Task_ = std::make_shared<chanhqp::task::TaskSE3Equality>("se3_task");
        jointTask_ = std::make_shared<chanhqp::task::TaskJointPostureEquality>("joint_posture_task");
        scaTask_ = std::make_shared<chanhqp::task::TaskSCAInequality>("sca_task", true);
        baseFixTask_ = std::make_shared<chanhqp::task::TaskBaseFixEquality>("base_fix_task");
        hqp_data_ = std::make_shared<chanhqp::data::HQPData>();
        jointVelLimitTask_ = std::make_shared<chanhqp::task::TaskJointVelocityLimitInequality>("joint_vel_limit_task", false);

        // Joint Velocity Limit of UR5e
        joint_vel_limit_["upper"] = Eigen::VectorXd::Ones(state_.n_manip) * 3.0; // URDF : 3.141592
        joint_vel_limit_["lower"] = -Eigen::VectorXd::Ones(state_.n_manip) * 3.0; // URDF : 3.141592
        jointVelLimitTask_ -> setJointVelLimit(joint_vel_limit_);

    }

    SE3 SummitUR5Wrapper::oMi() {return state_.oMi;}
    Reference SummitUR5Wrapper::get_result() {return ref_;}
    void SummitUR5Wrapper::initialize()
    {
        // Initialize Global Vector
        state_.q.resize(state_.n_var);
        state_.v.resize(state_.n_var);
        state_.J.resize(6,state_.n_var);

        ref_.mobile_vel = state_.v.head(state_.n_mobile);
        ref_.manip_pos = state_.q.tail(state_.n_manip);


        // Gain Setting
        se3Task_ -> Kp(400);
        se3Task_ -> Kd(40);
        jointTask_ -> Kp(10);
    
    }

    void SummitUR5Wrapper::update_joint(Eigen::VectorXd q,  Eigen::VectorXd v)
    {
        // State
        state_.q = q;
        state_.v = v;
        robot_->computeAllTerms(data_, state_.q, state_.v);
        robot_->jacobianLocal(data_, model_.getJointId("ur5e_wrist_3_joint"), state_.J);
        state_.oMi = robot_->position(data_, model_.getJointId("ur5e_wrist_3_joint"));


        state_.eef_vel = state_.J * state_.v;
        state_.sca_gamma = summitUr5_scaManager_ -> forward_pass(state_.q.tail(state_.n_manip));
        state_.sca_dgamma_dq = summitUr5_scaManager_ -> numerical_jacobian_gamma(state_.q.tail(state_.n_manip));
    }

    void SummitUR5Wrapper::initJointPosture(VectorXd qTarget, ros::Duration duration)
    {
        // Task Remove
        hqp_data_ -> initalize_all_data();

        stime_ = ros::Time::now();
        duration_ =  duration;  

        ref_.manip_pos = state_.q.tail(state_.n_manip);
        ref_.mobile_vel = state_.v.head(state_.n_mobile);

        jointRef_ -> SetStartTime(ros::Time::now());
        jointRef_ -> SetInitSample(state_.q.tail(state_.n_manip));
        jointRef_ -> SetGoalSample(qTarget);
        jointRef_ -> SetDuration(duration_);

        hqp_data_ -> addTask(scaTask_, 0);
        hqp_data_ -> addTask(jointVelLimitTask_, 0);
        hqp_data_ -> addTask(jointTask_, 1);
        // hqp_data_ -> addTask(baseFixTask_, 2);     

    }
    
    void SummitUR5Wrapper::jointPosture(ros::Time ctime)
    {
        jointRef_->SetCurrentTime(ctime);
        jointTask_ -> getVectorTraj(jointRef_ -> computeNext());
        hqp_data_ -> compute_problem_data(state_);
        Eigen::VectorXd qdot = hqp_data_ -> solve();

        ref_.mobile_vel = qdot.head(state_.n_mobile);
        ref_.manip_pos = state_.q.tail(state_.n_manip) + qdot.tail(state_.n_manip) * 0.01; 

        auto c = jointVelLimitTask_ -> constraint();
        bool joint_limit_ok = c->checkConstraint(qdot);

        // std::cout<<"[Controller] n_manip :"<<state_.n_manip<<std::endl;
        // std::cout<<"[Controller] manip_pos : "<<ref_.manip_pos.transpose()<<std::endl;
        // std::cout<<"[Controller] Joint Vel \n"<<qdot.tail(state_.n_manip).transpose()<<std::endl;
        // std::cout<<"[Controller] Joint Limit : "<<joint_limit_ok<<std::endl;

    }


    void SummitUR5Wrapper::initControl(SE3 target, ros::Duration duration, bool rel)
    {
        // Task Remove
        hqp_data_ -> initalize_all_data();

        stime_ = ros::Time::now();
        duration_ = duration;
        
        SE3 oMg;
        if(rel)
        {
            oMg.translation() = state_.oMi.translation() + target.translation();
            oMg.rotation() = state_.oMi.rotation() * target.rotation();
        }
        else oMg = target;

        ref_.manip_pos = state_.q.tail(state_.n_manip);
        ref_.mobile_vel = state_.v.head(state_.n_mobile);

        se3Ref_ -> SetInitSample(state_.oMi);
        se3Ref_ -> SetGoalSample(oMg);
        se3Ref_ -> SetStartTime(stime_);
        se3Ref_ -> SetDuration(duration);

        // Task Insert
        hqp_data_ -> addTask(scaTask_, 0);
        hqp_data_ -> addTask(se3Task_, 1);
        hqp_data_ -> addTask(baseFixTask_, 2);        

    }

    void SummitUR5Wrapper::Control(ros::Time ctime)
    {
        se3Ref_ -> SetCurrentTime(ctime);    
        se3Task_ -> getSE3Traj(se3Ref_->computeNext());

        hqp_data_ -> compute_problem_data(state_);
        Eigen::VectorXd qdot = hqp_data_ -> solve();

        ref_.mobile_vel = qdot.head(state_.n_mobile);
        ref_.manip_pos = state_.q.tail(state_.n_manip) + qdot.tail(state_.n_manip) * 0.01; 

    }

}


