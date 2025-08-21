#include <rci_holistic_controller/task/task_se3_equality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskSE3Equality::TaskSE3Equality(std::string & name):
    TaskBase(name){};

const bool & TaskSE3Equality::getSE3Traj(pinocchio::SE3 trajSample) const
{
    ref_se3_ = trajSample;
    return true;
}

const bool & TaskSE3Equality::getVectorTraj(Eigen::VectorXd ) const
{
    assert(false);
    return false;
}

ConstraintBase & compute(State state)
{
    pinocchio::SE3 dMi = state.oMi.inverse() * ref_se3_;
    pinocchio::Motion x_err_motion = pinocchio::log(dMi);
    Eigen::VectorXd x_err = x_err_motion.toVector();
    constraint_.setMatrix(state.J);
    constraint_.setVector(x_err);
    return constraint_;
}