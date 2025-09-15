#include "rci_action_manager/controller/controller.hpp"

using namespace Eigen;
using namespace pinocchio;
using namespace std;

using namespace holistic_controller::math;

namespace RobotController
{
    SummitUR5Wrapper::SummitUR5Wrapper(std::shared_ptr<holistic_controller::robot::RobotWrapper> robot)
    : robot_(robot)
    {
        model_ = robot_->model();
        Data data(model_);
        data_ = data;

        nq_ = model_.nq;
        nv_ = model_.nv;
        na_ = robot_->na();
        m_.resize(8);

        state_.n_var = nv_;

        ref_.mobile_vel.resize(3);
        ref_.manip_pos.resize(nq_-3);


    }

    SE3 SummitUR5Wrapper::oMi() {return oMi_;}
    Reference SummitUR5Wrapper::get_result() {return ref_;}
    void SummitUR5Wrapper::initialize()
    {
        q_.resize(nq_);
        v_.resize(nq_);
        u_.resize(nq_);
        J_.resize(6,nv_);

        ref_.mobile_vel = v_.head(3);
        ref_.manip_pos = q_.tail(6);

        solver_ = std::make_shared<holistic_controller::solver::HQP_solver>(15);
        se3Ref_ = std::make_shared<holistic_controller::trajectory::SE3CubicTrajectory>("SE3_Trajectory");
        jointRef_ = std::make_shared<holistic_controller::trajectory::JointCubicTrajectory>("SE3_Trajectory", nq_-3);
        summitUr5_scaManager_ = std::make_shared<rci_summit_ur5_controller::SummitUR5_ScaManager>();

        // Test : Task Tracking -> OK
        se3Task_ = std::make_shared<chanhqp::task::TaskSE3Equality>("se3_task");
        se3Task_ -> Kp(100);
        se3Task_ -> Kd(40);

        // Test : Task SCA -> OK
        scaTask_ = std::make_shared<chanhqp::task::TaskSCAInequality>("sca_task");

        // Test : Task Base Fix -> OK
        baseFixTask_ = std::make_shared<chanhqp::task::TaskBaseFixEquality>("base_fix_task");
        

        // Test : HQP Data
        hqp_data_ = std::make_shared<chanhqp::data::HQPData>();
        
    
    }

    void SummitUR5Wrapper::update_joint(Eigen::VectorXd q,  Eigen::VectorXd v)
    {
        q_ = q;
        v_ = v;
        robot_->computeAllTerms(data_, q_, v_);
        robot_->jacobianLocal(data_, model_.getJointId("ur5e_wrist_3_joint"), J_);
        oMi_ = robot_->position(data_, model_.getJointId("ur5e_wrist_3_joint"));


        // State
        state_.J = J_;
        state_.oMi = oMi_;
        state_.eef_vel = J_ * v_;
        state_.sca_gamma = summitUr5_scaManager_ -> forward_pass(q_.tail(6));
        state_.sca_dgamma_dq = summitUr5_scaManager_ -> numerical_jacobian_gamma(q_.tail(6));


    }

    void SummitUR5Wrapper::initJointPosture(VectorXd qTarget, ros::Duration duration)
    {
        stime_ = ros::Time::now();
        duration_ =  duration;  
        cout<<"Controller : "<<qTarget.transpose()<<endl;
        ref_.manip_pos = q_.tail(nq_-3);
        ref_.mobile_vel = v_.head(3);

        jointRef_ -> SetStartTime(ros::Time::now());
        jointRef_ -> SetInitSample(q_.tail(6));
        jointRef_ -> SetGoalSample(qTarget);
        jointRef_ -> SetDuration(duration_);



    }

    void SummitUR5Wrapper::jointPosture(ros::Time ctime)
    {
        jointRef_->SetCurrentTime(ctime);
        // q_des_.tail(nq_-3) = jointRef_->computeNext();

        VectorXd tmpq = q_;
        tmpq.tail(nq_-3) = jointRef_ ->computeNext();

        Eigen::VectorXd q_des_dot(6);
        q_des_dot = (tmpq.tail(nq_-3) - q_.tail(nq_-3))/0.1;

        // TEST 
        hqp_data_ -> addTask(scaTask_, 1);
        hqp_data_ -> compute_problem_data(state_);
        auto constraint = hqp_data_ -> get_constraint();
        chanhqp::constraint::ConstraintBase& constraint_sca = *constraint.at(1);

    //    // TEST SCA //
    //    double gamma_q = summitUr5_scaManager_ -> forward_pass(q_.tail(6));
    //    MatrixXd derivate_gamma_q = summitUr5_scaManager_ -> numerical_jacobian_gamma(q_.tail(6));

        Eigen::VectorXi activeSet(0);
        size_t activeSetSize;

        // // HQP
        MatrixXd Q_1(11,11);
        VectorXd C_1(11);
        Q_1.setIdentity();
        C_1.setZero();
        Q_1.topLeftCorner(9,9) *= 1e-5;
        MatrixXd A_eq1(0, 11);
        VectorXd B_eq1(0);
        MatrixXd A_ineq1(2,11);
        VectorXd B_ineq1(2);
        A_ineq1.topLeftCorner(2,9) = constraint_sca.getMatrix();
        // A_ineq1(0,9) = 1; // Slack
        A_ineq1.topRightCorner(2,2).setIdentity();
        B_ineq1 = constraint_sca.getVector();
        VectorXd x_1(11);

        double out1 = eiquadprog::solvers::solve_quadprog(Q_1, C_1, A_eq1.transpose(), -B_eq1,A_ineq1.transpose(), B_ineq1, x_1, activeSet, activeSetSize);
        VectorXd slack_1 = x_1.tail(2);
        ROS_WARN_STREAM("slack1 :"<<slack_1.transpose());
        ROS_WARN_STREAM("check const1 :"<<constraint_sca.checkConstraint(x_1.head(9)));
        
        MatrixXd Q_2(12,12);
        VectorXd C_2(12);
        Q_2.setIdentity();
        C_2.setZero();
        Q_2.topLeftCorner(9,9) *= 1e-5;
        MatrixXd A_eq2(3, 12);
        VectorXd B_eq2(3);
        A_eq2.setZero();
        A_eq2.topLeftCorner(3,3).setIdentity();
        A_eq2.topRightCorner(3,3).setIdentity();
        B_eq2.setZero();
        MatrixXd A_ineq2(2,12);
        VectorXd B_ineq2(2);
        A_ineq2.setZero();
        A_ineq2.topLeftCorner(2,9) = constraint_sca.getMatrix();
        B_ineq2 = B_ineq1 - slack_1;

        VectorXd x_2(12);

        double out2 = eiquadprog::solvers::solve_quadprog(Q_2, C_2, A_eq2.transpose(), -B_eq2,A_ineq2.transpose(), B_ineq2, x_2, activeSet, activeSetSize);
        VectorXd slack_2 = x_2.tail(3);
        // cout<<"Slack 2 : "<<slack_2.transpose()<<endl;

        MatrixXd Q_3(15,15);
        VectorXd C_3(15);
        Q_3.setIdentity();
        C_3.setZero();
        Q_3.topLeftCorner(9,9) *= 1e-5;
        MatrixXd A_eq3(9, 15);
        VectorXd B_eq3(9);
        A_eq3.setZero();
        A_eq3.block<6,6>(0,3).setIdentity();
        A_eq3.topRightCorner(6,6).setIdentity();
        A_eq3.bottomLeftCorner(3,3).setIdentity();
        B_eq3.setZero();
        B_eq3.head(6) = q_des_dot;
        B_eq3.tail(3) = B_eq2 - slack_2;
        MatrixXd A_ineq3(2,15);
        VectorXd B_ineq3(2);
        A_ineq3.setZero();
        A_ineq3.topLeftCorner(2,9) = constraint_sca.getMatrix();
        B_ineq3 = B_ineq1 - slack_1;

        VectorXd x_3(15);

        double out3 = eiquadprog::solvers::solve_quadprog(Q_3, C_3, A_eq3.transpose(), -B_eq3,A_ineq3.transpose(), B_ineq3, x_3, activeSet, activeSetSize);
        VectorXd slack_3 = x_3.tail(6);
        // cout<<"Slack 3 : "<<slack_3.transpose()<<endl;

        // cout<<"gamma q : "<<gamma_q<<endl;
        // double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -B_eq,A_ineq.transpose(), B_ineq, x, activeSet, activeSetSize);

        VectorXd qdot = x_3.head(9);
        // ROS_WARN_STREAM("qdot :"<<qdot.transpose());
        
        ROS_INFO_STREAM("Constraint A :"<<constraint_sca.getMatrix());
        ROS_INFO_STREAM("Constraint B :"<<constraint_sca.getVector().transpose());
        ROS_WARN_STREAM("mobile :"<<state_.sca_gamma["mobile"]);
        ROS_WARN_STREAM("manip :"<<state_.sca_gamma["manip"]);
        ROS_WARN_STREAM("check const :"<<constraint_sca.checkConstraint(qdot));

        // q_des_ = q_des_ + qdot * 0.01;
        ref_.mobile_vel = qdot.head(3);
        ref_.manip_pos = ref_.manip_pos + qdot.tail(nq_-3) * 0.01;

        // ref_.mobile_vel = Eigen::VectorXd::Zero(3);
        // ref_.manip_pos = ref_.manip_pos + qdot.tail(nq_-3);
    }


    void SummitUR5Wrapper::initControl(SE3 target, ros::Duration duration, bool rel)
    {
        stime_ = ros::Time::now();
        duration_ = duration;
        
        SE3 oMg;
        if(rel)
        {
            oMg.translation() = oMi_.translation() + target.translation();
            oMg.rotation() = oMi_.rotation() * target.rotation();
        }
        else oMg = target;

    

        cout<<"oMi \n"<<oMi_<<endl;
        cout<<"oMg \n"<<oMg<<endl;

        q_des_ = q_;

        ref_.manip_pos = q_.tail(nq_-3);
        ref_.mobile_vel = v_.head(3);

        se3Ref_ -> SetInitSample(oMi_);
        se3Ref_ -> SetGoalSample(oMg);
        se3Ref_ -> SetStartTime(stime_);
        se3Ref_ -> SetDuration(duration);
    }

    // hard coding
    // void SummitUR5Wrapper::Control(ros::Time ctime)
    // {

    //     se3Ref_ -> SetCurrentTime(ctime);
    //     SE3 oMi_ref = se3Ref_ -> computeNext();
    //     SE3 dMi = oMi_.inverse() * oMi_ref;
    //     Motion x_err = log6(dMi);
        
    //     VectorXd des_xdot = 0.1 * x_err.toVector();
    //     // MatrixXd Jinv = pseudoinv(J_);
    //     // VectorXd qdot = Jinv * (des_xdot);

    //     // TEST SCA //
    //     double gamma_q = summitUr5_scaManager_ -> forward_pass(q_.tail(6));
    //     MatrixXd derivate_gamma_q = summitUr5_scaManager_ -> numerical_jacobian_gamma(q_.tail(6));



    //     Eigen::VectorXi activeSet(0);
    //     size_t activeSetSize;

    //     // HQP
    //     MatrixXd Q_1(10,10);
    //     VectorXd C_1(10);
    //     Q_1.setIdentity();
    //     C_1.setZero();
    //     Q_1.topLeftCorner(9,9) *= 1e-8;
    //     MatrixXd A_eq1(0, 10);
    //     VectorXd B_eq1(0);
    //     MatrixXd A_ineq1(1,10);
    //     VectorXd B_ineq1(1);
    //     A_ineq1.setZero();
    //     A_ineq1.block<1,6>(0,3) = derivate_gamma_q;
    //     A_ineq1(0,9) = 1;
    //     B_ineq1(0) = 0.1 * std::log(gamma_q+1);
    //     VectorXd x_1(10);

    //     double out1 = eiquadprog::solvers::solve_quadprog(Q_1, C_1, A_eq1.transpose(), -B_eq1,A_ineq1.transpose(), B_ineq1, x_1, activeSet, activeSetSize);
    //     VectorXd slack_1 = x_1.tail(1);



    //     MatrixXd Q_2(12,12);
    //     VectorXd C_2(12);
    //     Q_2.setIdentity();
    //     C_2.setZero();
    //     Q_2.topLeftCorner(9,9) *= 1e-8;
    //     MatrixXd A_eq2(3, 12);
    //     VectorXd B_eq2(3);
    //     A_eq2.setZero();
    //     A_eq2.topLeftCorner(3,3).setIdentity();
    //     A_eq2.topRightCorner(3,3).setIdentity();
    //     B_eq2.setZero();
    //     MatrixXd A_ineq2(1,12);
    //     VectorXd B_ineq2(1);
    //     A_ineq2.setZero();
    //     A_ineq2.block<1,6>(0,3) = derivate_gamma_q;
    //     B_ineq2(0) =0.1 * std::log(gamma_q+1);
    //     B_ineq2 = B_ineq1 - slack_1;

    //     VectorXd x_2(12);

    //     double out2 = eiquadprog::solvers::solve_quadprog(Q_2, C_2, A_eq2.transpose(), -B_eq2,A_ineq2.transpose(), B_ineq2, x_2, activeSet, activeSetSize);
    //     VectorXd slack_2 = x_2.tail(3);


    //     MatrixXd Q_3(15,15);
    //     VectorXd C_3(15);
    //     Q_3.setIdentity();
    //     C_3.setZero();
    //     Q_3.topLeftCorner(9,9) *= 1e-8;
    //     MatrixXd A_eq3(9, 15);
    //     VectorXd B_eq3(9);
    //     A_eq3.setZero();
    //     A_eq3.topLeftCorner(6,9) = J_;
    //     A_eq3.topRightCorner(6,6).setIdentity();
    //     A_eq3.bottomLeftCorner(3,3).setIdentity();
    //     B_eq3.setZero();
    //     B_eq3.head(6) = 10 * des_xdot;
    //     B_eq3.tail(3) = B_eq2 - slack_2;
    //     MatrixXd A_ineq3(1,15);
    //     VectorXd B_ineq3(1);
    //     A_ineq3.setZero();
    //     A_ineq3.block<1,6>(0,3) = derivate_gamma_q;
    //     B_ineq3(0) = 0.1 * std::log(gamma_q+1);
    //     B_ineq3 = B_ineq1 - slack_1;

    //     VectorXd x_3(15);

    //     double out3 = eiquadprog::solvers::solve_quadprog(Q_3, C_3, A_eq3.transpose(), -B_eq3,A_ineq3.transpose(), B_ineq3, x_3, activeSet, activeSetSize);
    //     VectorXd qdot = x_3.head(9);
 
    //     // VectorXd slack_3 = x_3.tail(6);

    //     // MatrixXd Q_4(9,9);
    //     // VectorXd C_4(9);
    //     // Q_4.setIdentity();
    //     // Q_4.topLeftCorner(3,3) *= 1e-5;
    //     // C_4.setZero();
    //     // MatrixXd A_eq4(9, 9);
    //     // VectorXd B_eq4(9);
    //     // A_eq4.setZero();
    //     // A_eq4.topLeftCorner(6,9) = J_;
    //     // A_eq4.topRightCorner(6,6).setIdentity();
    //     // A_eq4.bottomLeftCorner(3,3).setIdentity();
    //     // B_eq4.setZero();
    //     // B_eq4.head(6) = 10 * des_xdot - slack_3;
    //     // B_eq4.tail(3) = B_eq2 - slack_2;
    //     // MatrixXd A_ineq4(1,9);
    //     // VectorXd B_ineq4(1);
    //     // A_ineq4.setZero();
    //     // A_ineq4.block<1,6>(0,3) = derivate_gamma_q;
    //     // B_ineq4(0) = 0.1 * std::log(gamma_q+1);
    //     // B_ineq4 = B_ineq1 - slack_1;
    //     // VectorXd x_4(9);

    //     // double out4 = eiquadprog::solvers::solve_quadprog(Q_4, C_4, A_eq4.transpose(), -B_eq4,A_ineq4.transpose(), B_ineq4, x_4, activeSet, activeSetSize);
    //     // // double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -B_eq,A_ineq.transpose(), B_ineq, x, activeSet, activeSetSize);

    //     // VectorXd qdot = x_4.head(9);



    //     q_des_ = q_des_ + qdot * 0.01;
    // }

    void SummitUR5Wrapper::Control(ros::Time ctime)
    {

        se3Ref_ -> SetCurrentTime(ctime);
        SE3 oMi_ref = se3Ref_ -> computeNext();
        
        se3Task_ -> getSE3Traj(oMi_ref);

        // TEST 
        hqp_data_ -> addTask(se3Task_, 2);
        hqp_data_ -> addTask(scaTask_, 1);
        hqp_data_ -> addTask(baseFixTask_, 3);        
        hqp_data_ -> compute_problem_data(state_);

        auto constraint = hqp_data_ -> get_constraint();

        // OK
        chanhqp::constraint::ConstraintBase& constraint_tracking = *constraint.at(2);
        chanhqp::constraint::ConstraintBase& constraint_sca = *constraint.at(1);
        chanhqp::constraint::ConstraintBase& constraint_base_fix = *constraint.at(3);

        // HQP
        Eigen::VectorXi activeSet(0);
        size_t activeSetSize;
        MatrixXd Q_1(11,11);
        VectorXd C_1(11);
        Q_1.setIdentity();
        C_1.setZero();
        Q_1.topLeftCorner(9,9) *= 1e-8;
        MatrixXd A_eq1(0, 11);
        VectorXd B_eq1(0);
        MatrixXd A_ineq1(2,11);
        VectorXd B_ineq1(2);
        A_ineq1.topLeftCorner(2,9) = constraint_sca.getMatrix();
        // A_ineq1(0,9) = 1; // Slack
        A_ineq1.topRightCorner(2,2).setIdentity();
        B_ineq1 = constraint_sca.getVector();
        VectorXd x_1(11);

        double out1 = eiquadprog::solvers::solve_quadprog(Q_1, C_1, A_eq1.transpose(), -B_eq1,A_ineq1.transpose(), B_ineq1, x_1, activeSet, activeSetSize);
        VectorXd slack_1 = x_1.tail(2);



        MatrixXd Q_2(15,15);
        VectorXd C_2(15);
        Q_2.setIdentity();
        C_2.setZero();
        Q_2.topLeftCorner(9,9) *= 1e-8;
        MatrixXd A_eq2(6, 15);
        VectorXd B_eq2(6);
        A_eq2.setZero();
        A_eq2.topLeftCorner(6,9) = constraint_tracking.getMatrix();;
        A_eq2.topRightCorner(6,6).setIdentity();
        B_eq2 = constraint_tracking.getVector();

        MatrixXd A_ineq2(2,15);
        VectorXd B_ineq2(2);
        A_ineq2.setZero();
        A_ineq2.topLeftCorner(2, 9) = constraint_sca.getMatrix();
        B_ineq2 = B_ineq1 - slack_1;

        VectorXd x_2(12);

        double out2 = eiquadprog::solvers::solve_quadprog(Q_2, C_2, A_eq2.transpose(), -B_eq2,A_ineq2.transpose(), B_ineq2, x_2, activeSet, activeSetSize);
        VectorXd slack_2 = x_2.tail(6);


        MatrixXd Q_3(15,15);
        VectorXd C_3(15);
        Q_3.setIdentity();
        C_3.setZero();
        Q_3.topLeftCorner(9,9) *= 1e-8;
        MatrixXd A_eq3(12, 15);
        VectorXd B_eq3(12);
        A_eq3.setZero();
        A_eq3.topLeftCorner(6,9) = constraint_base_fix.getMatrix();;
        A_eq3.topRightCorner(6,6).setIdentity();
        A_eq3.bottomLeftCorner(6,9) = constraint_tracking.getMatrix();
        B_eq3.setZero();
        B_eq3.head(6) = constraint_base_fix.getVector();
        B_eq3.tail(6) = B_eq2 - slack_2;
        MatrixXd A_ineq3(2,15);
        VectorXd B_ineq3(2);
        A_ineq3.setZero();
        A_ineq3.topLeftCorner(2,9) = constraint_sca.getMatrix();
        B_ineq3 = B_ineq1 - slack_1;

        VectorXd x_3(15);

        double out3 = eiquadprog::solvers::solve_quadprog(Q_3, C_3, A_eq3.transpose(), -B_eq3,A_ineq3.transpose(), B_ineq3, x_3, activeSet, activeSetSize);
        VectorXd qdot = x_3.head(9);

        ROS_INFO_STREAM("Constraint A :"<<constraint_sca.getMatrix());
        ROS_INFO_STREAM("Constraint B :"<<constraint_sca.getVector().transpose());
        ROS_WARN_STREAM("mobile :"<<state_.sca_gamma["mobile"]);
        ROS_WARN_STREAM("manip :"<<state_.sca_gamma["manip"]);
        ROS_WARN_STREAM("check const :"<<constraint_sca.checkConstraint(qdot));
        
    //     // VectorXd slack_3 = x_3.tail(6);

    //     // MatrixXd Q_4(9,9);
    //     // VectorXd C_4(9);
    //     // Q_4.setIdentity();
    //     // Q_4.topLeftCorner(3,3) *= 1e-5;
    //     // C_4.setZero();
    //     // MatrixXd A_eq4(9, 9);
    //     // VectorXd B_eq4(9);
    //     // A_eq4.setZero();
    //     // A_eq4.topLeftCorner(6,9) = J_;
    //     // A_eq4.topRightCorner(6,6).setIdentity();
    //     // A_eq4.bottomLeftCorner(3,3).setIdentity();
    //     // B_eq4.setZero();
    //     // B_eq4.head(6) = 10 * des_xdot - slack_3;
    //     // B_eq4.tail(3) = B_eq2 - slack_2;
    //     // MatrixXd A_ineq4(1,9);
    //     // VectorXd B_ineq4(1);
    //     // A_ineq4.setZero();
    //     // A_ineq4.block<1,6>(0,3) = derivate_gamma_q;
    //     // B_ineq4(0) = 0.1 * std::log(gamma_q+1);
    //     // B_ineq4 = B_ineq1 - slack_1;
    //     // VectorXd x_4(9);

    //     // double out4 = eiquadprog::solvers::solve_quadprog(Q_4, C_4, A_eq4.transpose(), -B_eq4,A_ineq4.transpose(), B_ineq4, x_4, activeSet, activeSetSize);
    //     // // double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -B_eq,A_ineq.transpose(), B_ineq, x, activeSet, activeSetSize);

    //     // VectorXd qdot = x_4.head(9);


        // QP
        // Eigen::VectorXi activeSet(0);q_des_
        // size_t activeSetSize;
        // MatrixXd Q_1(15,15);
        // VectorXd C_1(15);
        // Q_1.setIdentity();
        // Q_1.topLeftCorner(9,9) *= 1e-5;
        // C_1.setZero();
        // MatrixXd A_eq1(6, 15);
        // VectorXd B_eq1(6);
        // MatrixXd A_ineq1(0,15);
        // VectorXd B_ineq1(0);

        // VectorXd x_1(9);

        // A_eq1 = A_eq;
        // B_eq1 = B_eq;
        // double out1 = eiquadprog::solvers::solve_quadprog(Q_1, C_1, A_eq1.transpose(), -B_eq1,A_ineq1.transpose(), B_ineq1, x_1, activeSet, activeSetSize);
        // VectorXd slack_1 = x_1.tail(1);
        
        // q_des_ = q_des_ + qdot * 0.01;
        ref_.mobile_vel = qdot.head(3);
        ref_.manip_pos = q_.tail(nq_-3) + qdot.tail(nq_-3) * 0.01; 

        // OK
        // std::cout<<"Constraint Test"<<std::endl;
        // std::cout<<"Is Equality : "<<constraint.isEquality()<<std::endl;


    }

}


