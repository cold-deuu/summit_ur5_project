#include "rci_holistic_controller/data/hqp_data.hpp"
#include <ros/ros.h>
#include <cmath>
#include <stdexcept>

using namespace chanhqp::data;

HQPData::HQPData(){

}
void HQPData::qp_initialize()
{
    // Initialize QP Problem
    Q.resize(0,0);
    C.resize(0);
    A_eq.resize(0,0);
    b_eq.resize(0);
    A_ineq.resize(0,0);
    b_ineq.resize(0);
    slack_eq_.clear();
    slack_ineq_.clear();
    slack_eq_.push_back(Eigen::VectorXd(0));
    slack_ineq_.push_back(Eigen::VectorXd(0));
}

void HQPData::initalize_all_data()
{
    tasks_.clear();
    safety_tasks_.clear();
    constraint_.clear();
    safety_constraint_.clear();
}

void HQPData::addTask(std::shared_ptr<TaskBase> task, int priority)
{
    if (priority < 0) throw std::invalid_argument("HQP priority must be nonnegative");
    if (priority == 0) safety_tasks_.push_back(task);
    else
    {
        const bool inserted = tasks_.emplace(priority, task).second;
        if (!inserted) throw std::invalid_argument("Duplicate task priority");
    }

    std::cout<<"[HQP] Task Name : "<<task->name()<<" with Priority : "<<priority<<" Added!"<<std::endl;
}

void HQPData::compute_problem_data(State state)
{
    constraint_.clear();
    safety_constraint_.clear();
    // 범위 기반 for
    for (const auto& task_ptr : safety_tasks_) {
        safety_constraint_.push_back(task_ptr->compute(state));
    }


    for (const auto& [priority, task_ptr] : tasks_) {
        // Safety Constrant : Joint Limit, Self Collisio, etc
        std::shared_ptr<chanhqp::constraint::ConstraintBase> c = task_ptr->compute(state);

        if(priority == 0)
        {
            assert(false && "Priority 0 is not allowed!");
        }
        else
        {
            const bool inserted = constraint_.emplace(priority, c).second;
            if (!inserted) throw std::invalid_argument("Duplicate constraint priority");
        }

    }

    if (constraint_.empty()) throw std::invalid_argument("HQP requires a task");
    auto max_priority = constraint_.rbegin()->first;
    num_task_ = static_cast<int>(constraint_.size());

    // std::cout<<"Max :"<<max_priority<<"Num Task :"<<num_task_<<std::endl;
    if (max_priority != num_task_ || constraint_.begin()->first != 1)
        throw std::invalid_argument("HQP task priorities must be consecutive from 1");

}


Eigen::VectorXd HQPData::solve()
{
    this->qp_initialize();
    if (constraint_.empty()) throw std::logic_error("Compute HQP problem data before solve");
    const int n_var = constraint_.begin()->second->getMatrix().cols();
    sol_ = Eigen::VectorXd::Zero(n_var);

    // A failed QP must never expose its invalid solution to the controller.
    const auto fail = [&]() -> Eigen::VectorXd {
        ROS_ERROR_THROTTLE(1.0, "HQP solve failed; commanding zero velocity");
        sol_.setZero(n_var);
        return sol_;
    };
    try
    {
        Eigen::VectorXi activeSet(0);
        size_t activeSetSize = 0;


        bool check_eq_init = false;
        bool check_ineq_init = false;

        for(int i=0; i<safety_constraint_.size(); i++)
        {
            std::shared_ptr<chanhqp::constraint::ConstraintBase> cs = safety_constraint_[i];
            if (cs->isEquality())
            {
                int n_row_old = A_eq.rows();
                int n_col_old = A_eq.cols();
                int n_row_new = cs->getMatrix().rows();
                int n_col_new = cs->getMatrix().cols();
            
                Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
                Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
                mat.topLeftCorner(n_row_old, n_col_old) = A_eq;
                mat.bottomRightCorner(n_row_new, n_col_new) = cs->getMatrix();
                vec.head(n_row_old) = b_eq;
                vec.tail(n_row_new) = cs->getVector();

                A_eq = mat;
                b_eq = vec;

                check_eq_init = true;

            }
            else if(cs->isInequality())
            {
                int n_row_old = A_ineq.rows();
                int n_col_old = A_ineq.cols();
                int n_row_new = cs->getMatrix().rows();
                int n_col_new = cs->getMatrix().cols();  
            
                Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
                Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
                mat.topLeftCorner(n_row_old, n_col_old) = A_ineq;
                mat.bottomRightCorner(n_row_new, n_col_new) = cs->getMatrix();
                vec.head(n_row_old) = b_ineq;
                vec.tail(n_row_new) = cs->getVector();

                A_ineq = mat;
                b_ineq = vec;

                check_ineq_init = true;
            }
        }

        for(int i=1; i<num_task_+1; i++)
        {
            std::shared_ptr<chanhqp::constraint::ConstraintBase> c = constraint_.at(i);
            int n_row_new = c->getMatrix().rows();
            int n_col_new = c->getMatrix().cols();
            int n_slack = n_row_new;

            // Check Initialize for QP
            if(i==1)
            {
                if(!check_eq_init)
                {
                    A_eq.resize(0, n_col_new);
                    check_eq_init = true;
                }
                if(!check_ineq_init)
                {
                    A_ineq.resize(0, n_col_new);
                    check_ineq_init = true;
                }
            }

            if (c->isEquality())
            {
                int n_row_old = A_eq.rows();
                Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new + n_slack);
                Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
                Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
                Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
                C.setZero(n_col_new + n_slack);

                mat.topLeftCorner(n_row_old, n_col_new) = A_eq.topLeftCorner(n_row_old, n_col_new);
                mat.bottomLeftCorner(n_row_new, n_col_new) = c->getMatrix();
                mat.bottomRightCorner(n_row_new, n_slack).setIdentity();
                vec.head(n_row_old) = b_eq;
                vec.tail(n_row_new) = c->getVector();

                A_eq = mat;
                b_eq = vec;

                A_ineq.conservativeResize(A_ineq.rows(), n_col_new + n_slack);
                A_ineq.rightCols(n_slack).setZero();
                Eigen::VectorXd x(n_col_new + n_slack);

                if (!A_eq.allFinite() || !b_eq.allFinite() ||
                    !A_ineq.allFinite() || !b_ineq.allFinite()) return fail();
                activeSet.resize(0);
                activeSetSize = 0;
                double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);
                if (!std::isfinite(out) || !x.allFinite()) return fail();
                sol_ = x.head(n_col_new);
                slack_eq_.push_back(x.tail(n_slack));
                slack_ineq_.push_back(Eigen::VectorXd(0));
                // Freeze this level's residual before solving any lower priority.
                b_eq.tail(n_slack) -= slack_eq_.back();
                A_eq = A_eq.leftCols(n_col_new).eval();
                A_ineq = A_ineq.leftCols(n_col_new).eval();


            }
            else if(c->isInequality())
            {
                int n_row_old = A_ineq.rows();
                Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new + n_slack, n_col_new + n_slack);
                Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new + n_slack);
            
                Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
                Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
                C.setZero(n_col_new + n_slack);

                mat.topLeftCorner(n_row_old, n_col_new) = A_ineq.topLeftCorner(n_row_old, n_col_new);
                mat.block(n_row_old, 0, n_row_new, n_col_new) = c->getMatrix();
                // A*x + b + s >= 0, s >= 0.
                mat.block(n_row_old, n_col_new, n_slack, n_slack).setIdentity();
                mat.bottomRightCorner(n_slack, n_slack).setIdentity();
                vec.head(n_row_old) = b_ineq;
                vec.segment(n_row_old, n_row_new) = c->getVector();

                A_ineq = mat;
                b_ineq = vec;

                A_eq.conservativeResize(A_eq.rows(), n_col_new + n_slack);
                A_eq.rightCols(n_slack).setZero();

                Eigen::VectorXd x(n_col_new + n_slack);
                if (!A_eq.allFinite() || !b_eq.allFinite() ||
                    !A_ineq.allFinite() || !b_ineq.allFinite()) return fail();
                activeSet.resize(0);
                activeSetSize = 0;
                double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);

                if (!std::isfinite(out) || !x.allFinite()) return fail();
                sol_ = x.head(n_col_new);
                slack_ineq_.push_back(x.tail(n_slack));
                slack_eq_.push_back(Eigen::VectorXd(0));

                // Keep the relaxed bound; discard temporary slack positivity rows.
                b_ineq.segment(n_row_old, n_row_new) += slack_ineq_.back();
                A_ineq = A_ineq.topLeftCorner(n_row_old + n_row_new, n_col_new).eval();
                b_ineq = b_ineq.head(n_row_old + n_row_new).eval();
                A_eq = A_eq.leftCols(n_col_new).eval();
            }
        }
        return sol_;
    }
    catch (const std::exception &)
    {
        return fail();
    }
}


// Eigen::VectorXd HQPData::solve()
// {
//     this->qp_initialize();   
//     Eigen::VectorXi activeSet(0);
//     size_t activeSetSize;

//     for(int i=0; i<safety_constraint_.size(); i++)
//     {
//         std::shared_ptr<chanhqp::constraint::ConstraintBase> cs = safety_constraint_[i];
//         if (cs->isEquality())
//         {
//             int n_row_old = A_eq.rows();
//             int n_col_old = A_eq.cols();
//             int n_row_new = cs->getMatrix().rows();
//             int n_col_new = cs->getMatrix().cols();
            
//             Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
//             Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
//             mat.topLeftCorner(n_row_old, n_col_old) = A_eq;
//             mat.bottomRightCorner(n_row_new, n_col_new) = cs->getMatrix();
//             vec.head(n_row_old) = b_eq;
//             vec.tail(n_row_new) = cs->getVector();

//             A_eq = mat;
//             b_eq = vec;

//         }
//         else if(cs->isInequality())
//         {
//             int n_row_old = A_ineq.rows();
//             int n_col_old = A_ineq.cols();
//             int n_row_new = cs->getMatrix().rows();
//             int n_col_new = cs->getMatrix().cols();  
            
//             Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
//             Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
//             mat.topLeftCorner(n_row_old, n_col_old) = A_ineq;
//             mat.bottomRightCorner(n_row_new, n_col_new) = cs->getMatrix();
//             vec.head(n_row_old) = b_ineq;
//             vec.tail(n_row_new) = cs->getVector();

//             A_ineq = mat;
//             b_ineq = vec;

//         }
//     }


//     for(int i=1; i<num_task_+1; i++)
//     {
//         std::shared_ptr<chanhqp::constraint::ConstraintBase> c = constraint_.at(i);
//         int n_row_new = c->getMatrix().rows();
//         int n_col_new = c->getMatrix().cols();
//         int n_slack = n_row_new;

//         if (c->isEquality())
//         {
//             int n_row_old = A_eq.rows();
//             int n_col_old = A_eq.cols() - slack_eq_[i-1].size();

//             Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new + n_slack);
//             Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
//             Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
//             Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
//             C.setZero(n_col_new + n_slack);

//             mat.topLeftCorner(n_row_old, n_col_new) = A_eq.topLeftCorner(n_row_old, n_col_new);
//             mat.bottomLeftCorner(n_row_new, n_col_new) = c->getMatrix();
//             mat.bottomRightCorner(n_row_new, n_slack).setIdentity();
//             vec.head(n_row_old) = b_eq;
//             vec.tail(n_row_new) = c->getVector();

//             A_eq = mat;
//             b_eq = vec;

//             A_ineq.conservativeResize(A_ineq.rows(), n_col_new + n_slack);
//             A_ineq.rightCols(n_slack).setZero();
//             Eigen::VectorXd x(n_col_new + n_slack);

//             double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);

//             sol_ = x.head(n_col_new);
//             slack_eq_.push_back(x.tail(n_slack));
//             slack_ineq_.push_back(Eigen::VectorXd(0));
//             A_ineq = A_ineq.leftCols(n_col_new);
//         }
//         else if(c->isInequality())
//         {
//             int n_row_old = A_ineq.rows();
//             int n_col_old = A_ineq.cols() - slack_ineq_[i-1].size();

//             Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new + n_slack);
//             Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
//             Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
//             Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
//             C.setZero(n_col_new + n_slack);

//             mat.topLeftCorner(n_row_old, n_col_new) = A_ineq.topLeftCorner(n_row_old, n_col_new);
//             mat.bottomLeftCorner(n_row_new, n_col_new) = c->getMatrix();
//             vec.head(n_row_old) = b_ineq;
//             vec.tail(n_row_new) = c->getVector();

//             A_ineq = mat;
//             b_ineq = vec;

//             A_eq.conservativeResize(A_eq.rows(), n_col_new + n_slack);
//             A_eq.rightCols(n_slack).setZero();

//             Eigen::VectorXd x(n_col_new + n_slack);
//             double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);


//             sol_ = x.head(n_col_new);
//             slack_ineq_.push_back(x.tail(n_slack));
//             slack_eq_.push_back(Eigen::VectorXd(0));

//             A_eq = A_eq.leftCols(n_col_new);
//         }
//         // std::cout<<"[HQPData] Safety Ineq A : \n"<<A_ineq<<std::endl;
//         // std::cout<<"[HQPData] Safety Ineq b : \n"<<b_ineq.transpose()<<std::endl;
    
//         // i start from 1
//         b_eq.tail(slack_eq_[i-1].size()) -= slack_eq_[i-1];
//         b_ineq.tail(slack_ineq_[i-1].size()) -= slack_ineq_[i-1];
//     }
//     return sol_;
// }
// Fixed-size specialization of this same HQP for the six-joint topic controller.
// Priority 0: hard position-derived velocity, speed, acceleration and optional
// model-based torque bounds. Priority 1: qdot + slack = tracking reference.
// The existing epsilon regularization is a tie breaker, not a separate task.
#include <eiquadprog/eiquadprog-rt.hpp>
struct HQPData::JointTrackingWorkspace {
    eiquadprog::solvers::RtEiquadprog<12,6,24> solver;
    Eigen::Matrix<double,12,12> Q = Eigen::Matrix<double,12,12>::Identity();
    Eigen::Matrix<double,12,1> C = Eigen::Matrix<double,12,1>::Zero(), x;
    Eigen::Matrix<double,6,12> eq = Eigen::Matrix<double,6,12>::Zero();
    Eigen::Matrix<double,24,12> ineq = Eigen::Matrix<double,24,12>::Zero();
    Vector6 eq_offset;
    Eigen::Matrix<double,24,1> ineq_offset;
};
HQPData::~HQPData() = default;
void HQPData::prepareJointTracking(double regularization)
{
    joint_tracking_.reset(new JointTrackingWorkspace);
    auto &w = *joint_tracking_;
    w.Q.topLeftCorner<6,6>() *= regularization;
    w.eq.leftCols<6>().setIdentity();
    w.eq.rightCols<6>().setIdentity();
    w.ineq.topLeftCorner<6,6>().setIdentity();
    w.ineq.block<6,6>(6,0) = -Matrix6::Identity();
    w.solver.setMaxIter(100);
}
bool HQPData::solveJointTracking(const Vector6 &target, const Vector6 &lower,
    const Vector6 &upper, const Matrix6 &mass, const Vector6 &nonlinear,
    const Vector6 &previous_velocity, const Vector6 &effort, double dt,
    bool torque_limits, Vector6 &velocity)
{
    if (!joint_tracking_ || !target.allFinite() || !lower.allFinite() ||
        !upper.allFinite() || (lower.array()>upper.array()).any() ||
        !mass.allFinite() || !nonlinear.allFinite()) {
        velocity.setZero(); return false;
    }
    auto &w = *joint_tracking_;
    w.eq_offset = -target;
    w.ineq_offset.head<6>() = -lower;
    w.ineq_offset.segment<6>(6) = upper;
    if (torque_limits) {
        w.ineq.block<6,6>(12,0) = mass / dt;
        w.ineq.block<6,6>(18,0) = -mass / dt;
        const Vector6 bias = nonlinear - mass * previous_velocity / dt;
        w.ineq_offset.segment<6>(12) = effort + bias;
        w.ineq_offset.tail<6>() = effort - bias;
    } else {
        w.ineq.bottomRows<12>().setZero();
        w.ineq_offset.tail<12>().setOnes();
    }
    const auto status = w.solver.solve_quadprog(w.Q,w.C,w.eq,w.eq_offset,
                                               w.ineq,w.ineq_offset,w.x);
    if (status != eiquadprog::solvers::RT_EIQUADPROG_OPTIMAL || !w.x.allFinite()) {
        velocity.setZero(); return false;
    }
    velocity = w.x.head<6>();
    return true;
}
