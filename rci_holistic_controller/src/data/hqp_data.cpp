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
}
void HQPData::addTask(std::shared_ptr<TaskBase> task, int priority)
{
    auto [it, inserted] = tasks_.emplace(priority, task);
    assert(inserted && "Duplicate priority!");
}

void HQPData::compute_problem_data(State state)
{
    constraint_.clear();

    for (const auto& [priority, task_ptr] : tasks_) {
        auto [it, inserted] = constraint_.emplace(priority, &task_ptr->compute(state));
        assert(inserted && "Duplicate priority!");
    }

    assert(!constraint_.empty());
    auto max_priority = constraint_.rbegin()->first;
    num_task_ = static_cast<int>(constraint_.size());

    std::cout<<"Max :"<<max_priority<<"Num Task :"<<num_task_<<std::endl;
    assert(max_priority == num_task_);

}

// Eigen::VectorXd HQPData::solve()
// {
//     this->qp_initialize();   
//     Eigen::VectorXi activeSet(0);
//     size_t activeSetSize;

//     for(int i=1; i<num_task_+1; i++)
//     {
//         chanhqp::constraint::ConstraintBase& c = *constaint_.at(i);
//         Eigen::MatrixXd mat = c.getMatrix();
//         Eigen::VectorXd vec = c.getVector();
        
//         // num var and slacks
//         int n_var = mat.cols();
//         int n_slack = mat.rows();

//         if(c.isEquality())
//         {
//             int n_row = A_eq.rows();
//             int n_col = A_eq.cols();
            
//             Eigen::MatrixXd old_A_eq = A_eq.topLeftCorner(n);
//             Eigen::VectorXd olb_b_eq = b_eq;

//             A_eq = Eigen::MatrixXd::Zero(n_row + n_var, n_col + n_var + n_slack);
//             b_eq = Eigen::VectorXd::Zero(n_row + n_var)

//             A_eq.conservativeResize(n_row + n_var, n_col + n_var + n_slack);
//             b_eq.conservativeResize(n_row + n_slack);

//             A_eq.topRightCorner(n_slack, n_slack).setIdentity();
//         }


//     }

// }