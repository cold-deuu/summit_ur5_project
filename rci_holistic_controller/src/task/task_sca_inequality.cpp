#include <rci_holistic_controller/task/task_sca_inequality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskSCAInequality::TaskSCAInequality(const std::string & name):
    TaskBase(name), constraint_(name){};

bool TaskSCAInequality::getSE3Traj(pinocchio::SE3 trajSample) {
    assert(false);
    return false;
}

bool TaskSCAInequality::getVectorTraj(Eigen::VectorXd ) {
    assert(false);
    return false;
}

ConstraintBase & TaskSCAInequality::compute(State state)
{
    // Eigen::MatrixXd sca_ineq_mat(1,9);
    // Eigen::VectorXd sca_ineq_vec(1);
    // sca_ineq_mat.setZero();
    // sca_ineq_mat.block<1,6>(0,3) = state.sca_dgamma_dq;
    // sca_ineq_mat(0,9) = 1;
    // sca_ineq_vec(0) = 0.1 * std::log(state.sca_gamma+1);

    Eigen::MatrixXd sca_ineq_mat(2,9);
    Eigen::VectorXd sca_ineq_vec(2);
    sca_ineq_mat.setZero();
    sca_ineq_mat.block<1,6>(0,3) = state.sca_dgamma_dq["mobile"];
    // sca_ineq_mat(0,9) = 1;
    sca_ineq_vec(0) = 0.01 * std::log(state.sca_gamma["mobile"]+1);

    sca_ineq_mat.block<1,6>(1,3) = state.sca_dgamma_dq["manip"];
    // sca_ineq_mat(0,9) = 1;
    sca_ineq_vec(1) = 0.1 * std::log(state.sca_gamma["manip"]+1);

    constraint_.setMatrix(sca_ineq_mat);
    constraint_.setVector(sca_ineq_vec);
    return constraint_;
}