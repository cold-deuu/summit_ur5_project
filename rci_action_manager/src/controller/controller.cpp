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

        // Admittance Controller
        ft_sensor_.resize(6);
        Damping_ = Eigen::MatrixXd::Identity(6,6);
        Damping_.topLeftCorner(3,3) *= 0.005;
        Damping_.bottomRightCorner(3,3) *= 0.003;
        
        qdot_prev_.setZero(nv_);
        mass_.resize(nq_, nq_);

        // Admittance Controller V2
        des_twist_.setZero(6);
        M_a_ = MatrixXd::Identity(6,6);
        M_a_.topLeftCorner(3,3) *= 300.0;
        M_a_.bottomRightCorner(3,3) *= 10.0;
        D_a_ = MatrixXd::Identity(6,6);
        D_a_.topLeftCorner(3,3) *= 150.0;
        D_a_.bottomRightCorner(3,3) *= 15.0;

    }

    SE3 SummitUR5Wrapper::oMi() {return oMi_;}
    VectorXd SummitUR5Wrapper::get_result() {return desired_;}
    void SummitUR5Wrapper::initialize()
    {
        q_.resize(nq_);
        v_.resize(nq_);
        u_.resize(nq_);
        J_.resize(6,nv_);

        // Test //
        desired_.resize(nq_);


        solver_ = std::make_shared<holistic_controller::solver::HQP_solver>(15);
        se3Ref_ = std::make_shared<holistic_controller::trajectory::SE3CubicTrajectory>("SE3_Trajectory");
        jointRef_ = std::make_shared<holistic_controller::trajectory::JointCubicTrajectory>("SE3_Trajectory", nq_-3);
        summitUr5_scaManager_ = std::make_shared<rci_summit_ur5_controller::SummitUR5_ScaManager>();
    }

    void SummitUR5Wrapper::update_joint(Eigen::VectorXd q,  Eigen::VectorXd v)
    {
        qdot_prev_ = v_;
        q_ = q;
        v_ = v;
        robot_->computeAllTerms(data_, q_, v_);
        robot_->jacobianLocal(data_, model_.getJointId("ur5e_wrist_3_joint"), J_);
        oMi_ = robot_->position(data_, model_.getJointId("ur5e_wrist_3_joint"));

        // Admittance
        mass_ = robot_-> mass(data_);
        nle_ = robot_-> nonLinearEffects(data_);
    }

    void SummitUR5Wrapper::update_ft(Eigen::VectorXd ft)
    {
        // ft = -ft_global
        ft_sensor_ = ft;
    }

    void SummitUR5Wrapper::initJointPosture(VectorXd qTarget, ros::Duration duration)
    {
        stime_ = ros::Time::now();
        duration_ =  duration;  
        cout<<"Controller : "<<qTarget.transpose()<<endl;
        q_des_ = q_;

        jointRef_ -> SetStartTime(ros::Time::now());
        jointRef_ -> SetInitSample(q_.tail(6));
        jointRef_ -> SetGoalSample(qTarget);
        jointRef_ -> SetDuration(duration_);



    }

    void SummitUR5Wrapper::jointPosture(ros::Time ctime)
    {
        jointRef_->SetCurrentTime(ctime);
        q_des_.tail(nq_-3) = jointRef_->computeNext();
        desired_.setZero(); 
        desired_.tail(nq_-3) = q_des_.tail(nq_-3);

        // this->compute_wrench();

    //     VectorXd tmpq = q_;
    //     tmpq.tail(nq_-3) = jointRef_ ->computeNext();

    //     Eigen::VectorXd q_des_dot(6);
    //     q_des_dot = (tmpq.tail(nq_-3) - q_.tail(nq_-3))/1.0;

    

    //    // TEST SCA //
    //    double gamma_q = summitUr5_scaManager_ -> forward_pass(q_.tail(6));
    //    MatrixXd derivate_gamma_q = summitUr5_scaManager_ -> numerical_jacobian_gamma(q_.tail(6));

    //     Eigen::VectorXi activeSet(0);
    //     size_t activeSetSize;

    //     // HQP
    //     MatrixXd Q_1(10,10);
    //     VectorXd C_1(10);
    //     Q_1.setIdentity();
    //     C_1.setZero();
    //     Q_1.topLeftCorner(9,9) *= 1e-5;
    //     MatrixXd A_eq1(0, 10);
    //     VectorXd B_eq1(0);
    //     MatrixXd A_ineq1(1,10);
    //     VectorXd B_ineq1(1);
    //     A_ineq1.setZero();
    //     A_ineq1.block<1,6>(0,3) = derivate_gamma_q;
    //     A_ineq1(0,9) = 1;
    //     B_ineq1(0) = 0.01 * std::log(gamma_q+1);
    //     VectorXd x_1(10);

    //     double out1 = eiquadprog::solvers::solve_quadprog(Q_1, C_1, A_eq1.transpose(), -B_eq1,A_ineq1.transpose(), B_ineq1, x_1, activeSet, activeSetSize);
    //     VectorXd slack_1 = x_1.tail(1);
    //     cout<<"Slack 1 : "<<slack_1.transpose()<<endl;
        

    //     MatrixXd Q_2(12,12);
    //     VectorXd C_2(12);
    //     Q_2.setIdentity();
    //     C_2.setZero();
    //     Q_2.topLeftCorner(9,9) *= 1e-5;
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
    //     B_ineq2(0) = 0.01 * std::log(gamma_q+1);
    //     B_ineq2 = B_ineq1 - slack_1;

    //     VectorXd x_2(12);

    //     double out2 = eiquadprog::solvers::solve_quadprog(Q_2, C_2, A_eq2.transpose(), -B_eq2,A_ineq2.transpose(), B_ineq2, x_2, activeSet, activeSetSize);
    //     VectorXd slack_2 = x_2.tail(3);
    //     cout<<"Slack 2 : "<<slack_2.transpose()<<endl;

    //     MatrixXd Q_3(15,15);
    //     VectorXd C_3(15);
    //     Q_3.setIdentity();
    //     C_3.setZero();
    //     Q_3.topLeftCorner(9,9) *= 1e-5;
    //     MatrixXd A_eq3(9, 15);
    //     VectorXd B_eq3(9);
    //     A_eq3.setZero();
    //     A_eq3.block<6,6>(0,3).setIdentity();
    //     A_eq3.topRightCorner(6,6).setIdentity();
    //     A_eq3.bottomLeftCorner(3,3).setIdentity();
    //     B_eq3.setZero();
    //     B_eq3.head(6) = q_des_dot;
    //     B_eq3.tail(3) = B_eq2 - slack_2;
    //     MatrixXd A_ineq3(1,15);
    //     VectorXd B_ineq3(1);
    //     A_ineq3.setZero();
    //     A_ineq3.block<1,6>(0,3) = derivate_gamma_q;
    //     B_ineq3(0) = 0.01 * std::log(gamma_q+1);
    //     B_ineq3 = B_ineq1 - slack_1;

    //     VectorXd x_3(15);

    //     double out3 = eiquadprog::solvers::solve_quadprog(Q_3, C_3, A_eq3.transpose(), -B_eq3,A_ineq3.transpose(), B_ineq3, x_3, activeSet, activeSetSize);
    //     VectorXd slack_3 = x_3.tail(6);
    //     cout<<"Slack 3 : "<<slack_3.transpose()<<endl;

    //     cout<<"gamma q : "<<gamma_q<<endl;
    //     // double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -B_eq,A_ineq.transpose(), B_ineq, x, activeSet, activeSetSize);

    //     VectorXd qdot = x_3.head(9);

    //     q_des_ = q_des_ + qdot * 0.01;
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



        // cout<<"oMi \n"<<oMi_<<endl;
        // cout<<"oMg \n"<<oMg<<endl;

        q_des_ = q_;

        se3Ref_ -> SetInitSample(oMi_);
        se3Ref_ -> SetGoalSample(oMg);
        se3Ref_ -> SetStartTime(stime_);
        se3Ref_ -> SetDuration(duration);
    }

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
        SE3 dMi = oMi_.inverse() * oMi_ref;
        Motion x_err = log6(dMi);
        
        VectorXd des_xdot = 10.0 * x_err.toVector();
        // MatrixXd Jinv = pseudoinv(J_);
        // VectorXd qdot = Jinv * (des_xdot);

        // TEST SCA //
        double gamma_q = summitUr5_scaManager_ -> forward_pass(q_.tail(6));
        MatrixXd derivate_gamma_q = summitUr5_scaManager_ -> numerical_jacobian_gamma(q_.tail(6));



        Eigen::VectorXi activeSet(0);
        size_t activeSetSize;

        // HQP
        MatrixXd Q_1(10,10);
        VectorXd C_1(10);
        Q_1.setIdentity();
        C_1.setZero();
        Q_1.topLeftCorner(9,9) *= 1e-8;
        MatrixXd A_eq1(0, 10);
        VectorXd B_eq1(0);
        MatrixXd A_ineq1(1,10);
        VectorXd B_ineq1(1);
        A_ineq1.setZero();
        A_ineq1.block<1,6>(0,3) = derivate_gamma_q;
        A_ineq1(0,9) = 1;
        B_ineq1(0) = 0.01 * std::log(gamma_q+1);
        VectorXd x_1(10);

        double out1 = eiquadprog::solvers::solve_quadprog(Q_1, C_1, A_eq1.transpose(), -B_eq1,A_ineq1.transpose(), B_ineq1, x_1, activeSet, activeSetSize);
        VectorXd slack_1 = x_1.tail(1);



        // Tracking
        MatrixXd Q_2(15,15);
        VectorXd C_2(15);
        Q_2.setIdentity();
        C_2.setZero();
        Q_2.topLeftCorner(9,9) *= 1e-8;
        MatrixXd A_eq2(6, 15);
        VectorXd B_eq2(6);
        A_eq2.setZero();
        A_eq2.topLeftCorner(6,9) = J_;
        A_eq2.topRightCorner(6,6).setIdentity();
        B_eq2.setZero();
        B_eq2.head(6) = des_xdot;
    
        MatrixXd A_ineq2(1,15);
        VectorXd B_ineq2(1);
        A_ineq2.setZero();
        A_ineq2.block<1,6>(0,3) = derivate_gamma_q;
        B_ineq2(0) = 0.01 * std::log(gamma_q+1);
        B_ineq2 = B_ineq1 - slack_1;


        VectorXd x_2(15);

        double out2 = eiquadprog::solvers::solve_quadprog(Q_2, C_2, A_eq2.transpose(), -B_eq2,A_ineq2.transpose(), B_ineq2, x_2, activeSet, activeSetSize);
        VectorXd slack_2 = x_2.tail(6);
        VectorXd qdot = x_2.head(9);


        // // Base Fix
        // MatrixXd Q_3(12,12);
        // VectorXd C_3(12);
        // Q_3.setIdentity();
        // C_3.setZero();
        // Q_3.topLeftCorner(9,9) *= 1e-8;
        // MatrixXd A_eq3(9, 12);
        // VectorXd B_eq3(9);
        // A_eq3.setZero();
        // A_eq3.topLeftCorner(3,3).setIdentity();
        // A_eq3.topRightCorner(3,3).setIdentity();
        // A_eq3.bottomLeftCorner(6,9) = J_;
        // B_eq3.setZero();
        // B_eq3.tail(6) = B_eq2.head(6) - slack_2;
        // MatrixXd A_ineq3(1,12);
        // VectorXd B_ineq3(1);
        // A_ineq3.setZero();
        // A_ineq3.block<1,6>(0,3) = derivate_gamma_q;
        // B_ineq3(0) =0.01 * std::log(gamma_q+1);
        // B_ineq3 = B_ineq1 - slack_1;
        // VectorXd x_3(12);

        // double out3 = eiquadprog::solvers::solve_quadprog(Q_3, C_3, A_eq3.transpose(), -B_eq3,A_ineq3.transpose(), B_ineq3, x_3, activeSet, activeSetSize);
        // VectorXd qdot = x_3.head(9);
 
        // VectorXd slack_3 = x_3.tail(6);

        // MatrixXd Q_4(9,9);
        // VectorXd C_4(9);
        // Q_4.setIdentity();
        // Q_4.topLeftCorner(3,3) *= 1e-5;
        // C_4.setZero();
        // MatrixXd A_eq4(9, 9);
        // VectorXd B_eq4(9);
        // A_eq4.setZero();
        // A_eq4.topLeftCorner(6,9) = J_;
        // A_eq4.topRightCorner(6,6).setIdentity();
        // A_eq4.bottomLeftCorner(3,3).setIdentity();
        // B_eq4.setZero();
        // B_eq4.head(6) = 10 * des_xdot - slack_3;
        // B_eq4.tail(3) = B_eq2 - slack_2;
        // MatrixXd A_ineq4(1,9);
        // VectorXd B_ineq4(1);
        // A_ineq4.setZero();
        // A_ineq4.block<1,6>(0,3) = derivate_gamma_q;
        // B_ineq4(0) = 0.1 * std::log(gamma_q+1);
        // B_ineq4 = B_ineq1 - slack_1;
        // VectorXd x_4(9);

        // double out4 = eiquadprog::solvers::solve_quadprog(Q_4, C_4, A_eq4.transpose(), -B_eq4,A_ineq4.transpose(), B_ineq4, x_4, activeSet, activeSetSize);
        // // double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -B_eq,A_ineq.transpose(), B_ineq, x, activeSet, activeSetSize);

        // VectorXd qdot = x_4.head(9);

        // std::cout<<"oMg : "<<se3Ref_->m_goal<<std::endl;
        // std::cout<<"Time : "<<ctime.toSec() - stime_.toSec()<<std::endl;
        // std::cout<<"==========================="<<std::endl;

        q_des_ = q_des_ + qdot * 0.01;

        VectorXd desired(9);
        desired_.head(3) = qdot.head(3);
        desired_.tail(6) = q_des_.tail(6);
    
    }



    void SummitUR5Wrapper::initAdmittance()
    {
        init_admittance_ = oMi_;
    }

    void SummitUR5Wrapper::AdmittanceControl()
    {
        VectorXd ft_local = ft_sensor_;
        ft_local.head(3) = oMi_.rotation().transpose() * ft_local.head(3);
        ft_local.tail(3) = oMi_.rotation().transpose() * ft_local.tail(3);


        SE3 dMi = oMi_.inverse() * init_admittance_;
        VectorXd xdot_ref = log6(dMi).toVector() * 1;
        VectorXd xdot_admitt = Damping_ * ft_local;


        cout<<"X dot Admittance :"<<xdot_admitt.transpose()<<endl;

        VectorXd xdot_des = xdot_ref + xdot_admitt;
        // VectorXd xdot_des = xdot_ref;
        MatrixXd J_manip = J_.topRightCorner(6,6);
        MatrixXd Jinv = pseudoinv(J_manip);
        VectorXd qdot_ref = Jinv * xdot_des;

        desired_.head(3).setZero();
        // q_des_.tail(6) = q_des_.tail(6) + qdot_ref * 0.01;
        desired_.tail(6) = desired_.tail(6) + qdot_ref * 0.01;
    }


    void SummitUR5Wrapper::AdmittanceControl_v2()
    {
        Eigen::VectorXd ft_local = ft_sensor_;
        ft_local.head(3) = oMi_.rotation().transpose() * ft_local.head(3);
        ft_local.tail(3) = oMi_.rotation().transpose() * ft_local.tail(3);

        // // Compute Tracking Wrench
        // SE3 dMi = oMi_.inverse() * init_admittance_;
        // VectorXd xdot_ref = log6(dMi).toVector() * 1.0;
        // // xdot_ref.tail(3) *= 10.0;
        // MatrixXd J_manip = J_.topRightCorner(6,6);
        // MatrixXd Jinv = pseudoinv(J_manip);
        // VectorXd qdot_ref = Jinv * xdot_ref;
        // VectorXd qddot_des = qdot_ref * 10.0; // propotional control
        // VectorXd torque = mass_ * qddot_des;
        // MatrixXd J_t_inv = pseudoinv(J_manip.transpose());
        // VectorXd wrench_control = pseudoinv(J_manip.transpose()) * torque; 

        // Compute Tracking Wrench ver.2
        SE3 dMi = oMi_.inverse() * init_admittance_;
        VectorXd xddot_ref = log6(dMi).toVector() * 100.0 - J_.topRightCorner(6,6) * v_.tail(6) * 10.0;
        // xdot_ref.tail(3) *= 10.0;
        MatrixXd J_manip = J_.topRightCorner(6,6);
        MatrixXd Jinv = pseudoinv(J_manip);
        // VectorXd qdot_ref = Jinv * xdot_ref;
        VectorXd qddot_des = Jinv * xddot_ref;
        VectorXd torque = mass_ * qddot_des;
        MatrixXd J_t_inv = pseudoinv(J_manip.transpose());
        VectorXd wrench_control = pseudoinv(J_manip.transpose()) * torque; 





        VectorXd des_acc = VectorXd::Zero(6);
        des_acc = M_a_.inverse() * (-D_a_ * des_twist_ + ft_local + wrench_control);
        des_acc = des_acc.cwiseMax(-3.0).cwiseMin(3.0);

        des_twist_ += des_acc * 0.01;
        des_twist_ = des_twist_.cwiseMax(-3.0).cwiseMin(3.0);

        // IK
        VectorXd qdot_des = Jinv * des_twist_;
        desired_.head(3).setZero();
        // q_des_.tail(6) = q_des_.tail(6) + qdot_ref * 0.01;
        desired_.tail(6) = desired_.tail(6) + qdot_des * 0.01;

        // Debug
        // std::cout<<"ft_local : "<<ft_local.transpose()<<std::endl; // No Wrench : Zeros : OK
        std::cout<<"Wrench Ctrl :"<<wrench_control.transpose()<<std::endl; // Nan? Inf?
        // std::cout<<"Jt inv :\n "<<J_t_inv<<std::endl;
        // std::cout<<"Jt inv Det : "<<(J_t_inv * J_t_inv.transpose()).determinant()<<std::endl;
    }



    Eigen::VectorXd SummitUR5Wrapper::compute_wrench()
    {   
        VectorXd acc = (v_ - qdot_prev_)/0.01;
        VectorXd torque = mass_ * acc + nle_;
        VectorXd wrench = pseudoinv(J_.transpose()) * torque; 

        // std::cout<<"Wrench :"<<wrench.transpose()<<std::endl;
        // std::cout<<"FT Sensor :"<<ft_sensor_.transpose()<<std::endl;
        // std::cout<<"Wrench Norm :"<<wrench.norm()<<std::endl;
        // std::cout<<"FT Norm :"<<ft_sensor_.norm()<<std::endl;

        return wrench;
    }


}


