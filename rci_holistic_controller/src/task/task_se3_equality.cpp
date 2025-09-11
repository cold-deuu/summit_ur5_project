#include <rci_holistic_controller/task/task_se3_equality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskSE3Equality::TaskSE3Equality(const std::string & name):
    TaskBase(name), constraint_(name){};

bool TaskSE3Equality::getSE3Traj(pinocchio::SE3 trajSample) {
    ref_se3_ = trajSample;
    return true;
}

bool TaskSE3Equality::getVectorTraj(Eigen::VectorXd ) {
    assert(false);
    return false;
}

ConstraintBase & TaskSE3Equality::compute(State state)
{
    pinocchio::SE3 dMi = state.oMi.inverse() * ref_se3_;
    pinocchio::Motion x_err_motion = pinocchio::log6(dMi);
    Eigen::VectorXd x_err = x_err_motion.toVector();
    Eigen::VectorXd des_xdot = Kp_ * x_err;
    constraint_.setMatrix(state.J);
    constraint_.setVector(x_err);
    return constraint_;
}