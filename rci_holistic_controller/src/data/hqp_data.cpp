#include "rci_holistic_controller/data/hqp_data.hpp"

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
    // slack_eq_.resize(0);
    // slack_ineq_.resize(0);
    slack_eq_.push_back(Eigen::VectorXd(0));
    slack_ineq_.push_back(Eigen::VectorXd(0));
}
void HQPData::addTask(std::shared_ptr<TaskBase> task, int priority)
{
    auto [it, inserted] = tasks_.emplace(priority, task);
    std::cout<<"[HQP] Task Name : "<<task->name()<<" with Priority : "<<priority<<" Added!"<<std::endl;

    assert(inserted && "Duplicate priority!");
}

void HQPData::compute_problem_data(State state)
{
    constraint_.clear();
    safety_constraint_.clear();
    for (const auto& [priority, task_ptr] : tasks_) {
        // Safety Constrant : Joint Limit, Self Collisio, etc
        if(priority == 0)
        {
            safety_constraint_.push_back(&task_ptr->compute(state));
            // chanhqp::constraint::ConstraintBase& c = task_ptr->compute(state);
            // std::cout<<"[HQP] Safety Constraint Added!"<<std::endl;
            // std::cout<<"[HQP] Safety Constraint Matrix : "<<c.getMatrix()<<std::endl;
        }
        else
        {
            auto [it, inserted] = constraint_.emplace(priority, &task_ptr->compute(state));
            std::cout<<"[HQP] Task Priority : "<<priority<<" Added!"<<std::endl;
            std::cout<<"[HQP] Task Name : "<<task_ptr->name()<<std::endl;
        }
        assert(inserted && "Duplicate priority!");
    }

    assert(!constraint_.empty());
    auto max_priority = constraint_.rbegin()->first;
    num_task_ = static_cast<int>(constraint_.size());

    std::cout<<"Max :"<<max_priority<<"Num Task :"<<num_task_<<std::endl;
    assert(max_priority == num_task_);

}

Eigen::VectorXd HQPData::solve()
{
    this->qp_initialize();   
    Eigen::VectorXi activeSet(0);
    size_t activeSetSize;

    for(int i=0; i<safety_constraint_.size(); i++)
    {
        chanhqp::constraint::ConstraintBase& cs = *safety_constraint_[i];
        if (cs.isEquality())
        {
            int n_row_old = A_eq.rows();
            int n_col_old = A_eq.cols();
            int n_row_new = cs.getMatrix().rows();
            int n_col_new = cs.getMatrix().cols();
            
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            mat.topLeftCorner(n_row_old, n_col_old) = A_eq;
            mat.bottomRightCorner(n_row_new, n_col_new) = cs.getMatrix();
            vec.head(n_row_old) = b_eq;
            vec.tail(n_row_new) = cs.getVector();

            A_eq = mat;
            b_eq = vec;

            std::cout<<"[HQP] Safety Equality Constraint Added!"<<std::endl;
        }
        else if(cs.isInequality())
        {
            int n_row_old = A_ineq.rows();
            int n_col_old = A_ineq.cols();
            int n_row_new = cs.getMatrix().rows();
            int n_col_new = cs.getMatrix().cols();  
            
            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_new);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            mat.topLeftCorner(n_row_old, n_col_old) = A_ineq;
            mat.bottomRightCorner(n_row_new, n_col_new) = cs.getMatrix();
            vec.head(n_row_old) = b_ineq;
            vec.tail(n_row_new) = cs.getVector();

            A_ineq = mat;
            b_ineq = vec;
            std::cout<<"[HQP] Safety InEquality Constraint Added!"<<std::endl;

        }
    }

    
    for(int i=1; i<num_task_+1; i++)
    {
        chanhqp::constraint::ConstraintBase& c = *constraint_.at(i);
        std::cout<<"Constraint : "<<c.name()<<std::endl;
        int n_row_new = c.getMatrix().rows();
        int n_col_new = c.getMatrix().cols();
        int n_slack = n_row_new;

        if (c.isEquality())
        {
            int n_row_old = A_eq.rows();
            int n_col_old = A_eq.cols() - slack_eq_[i-1].size();

            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_old + n_slack);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
            Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
            C.setZero(n_col_new + n_slack);

            mat.topLeftCorner(n_row_old, n_col_old) = A_eq.topLeftCorner(n_row_old, n_col_old);
            mat.bottomLeftCorner(n_row_new, n_col_new) = c.getMatrix();
            mat.bottomRightCorner(n_row_new, n_slack).setIdentity();
            vec.head(n_row_old) = b_eq;
            vec.tail(n_row_new) = c.getVector();

            A_eq = mat;
            b_eq = vec;

            A_ineq.conservativeResize(A_ineq.rows(), n_col_new + n_slack);
            A_ineq.rightCols(n_slack).setZero();

            Eigen::VectorXd x(n_col_new + n_slack);
            double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);
            sol_ = x.head(n_col_new);
            slack_eq_.push_back(x.tail(n_slack));
            slack_ineq_.push_back(Eigen::VectorXd(0));

            A_ineq = A_ineq.leftCols(n_col_new);
        }
        else if(c.isInequality())
        {
            int n_row_old = A_ineq.rows();
            int n_col_old = A_ineq.cols() - slack_ineq_[i-1].size();

            Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row_old + n_row_new, n_col_old + n_col_new + n_slack);
            Eigen::VectorXd vec = Eigen::VectorXd::Zero(n_row_old + n_row_new);
            
            Q.setIdentity(n_col_new + n_slack, n_col_new + n_slack);
            Q.topLeftCorner(n_col_new, n_col_new) *= 1e-5;
            C.setZero(n_col_new + n_slack);

            mat.topLeftCorner(n_row_old, n_col_old) = A_ineq.topLeftCorner(n_row_old, n_col_old);
            mat.bottomLeftCorner(n_row_new, n_col_new) = c.getMatrix();
            vec.head(n_row_old) = b_ineq;
            vec.tail(n_row_new) = c.getVector();

            A_ineq = mat;
            b_ineq = vec;

            A_eq.conservativeResize(A_eq.rows(), n_col_new + n_slack);
            A_eq.rightCols(n_slack).setZero();

            Eigen::VectorXd x(n_col_new + n_slack);
            double out = eiquadprog::solvers::solve_quadprog(Q, C, A_eq.transpose(), -b_eq, A_ineq.transpose(), b_ineq, x, activeSet, activeSetSize);
            sol_ = x.head(n_col_new);
            slack_ineq_.push_back(x.tail(n_slack));
            slack_eq_.push_back(Eigen::VectorXd(0));

            A_eq = A_eq.leftCols(n_col_new);
            std::cout<<"[HQP] A_ineq1 : "<<c.getMatrix()<<std::endl;
            std::cout<<"[HQP] b_ineq1 : "<<c.getVector()<<std::endl;
        }

        // i start from 1
        b_eq.tail(slack_eq_[i-1].size()) -= slack_eq_[i-1];
        b_ineq.tail(slack_ineq_[i-1].size()) -= slack_ineq_[i-1];




    }
    std::cout<<"[HQP] A_ineq : "<<A_ineq<<std::endl;
    std::cout<<"[HQP] b_ineq : "<<b_ineq<<std::endl;
    std::cout<<"[HQP] QP Solved!"<<std::endl;
    std::cout<<"[HQP] Solution : "<<sol_.transpose()<<std::endl;
    return sol_;
}