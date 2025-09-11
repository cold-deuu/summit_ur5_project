#include <rci_holistic_controller/task/task_base_fix_equality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskBaseFixEquality::TaskBaseFixEquality(const std::string & name):
    TaskBase(name), constraint_(name){};

bool TaskBaseFixEquality::getSE3Traj(pinocchio::SE3 trajSample) {
    assert(false);
    return false;
}

bool TaskBaseFixEquality::getVectorTraj(Eigen::VectorXd ) {
    assert(false);
    return false;
}

ConstraintBase & TaskBaseFixEquality::compute(State state)
{
    Eigen::MatrixXd J_base = Eigen::MatrixXd::Zero(6,state.n_var);
    Eigen::VectorXd vel_base = Eigen::VectorXd::Zero(6);
    J_base(0,0) = J_base(1,1) = J_base(5,2) = 1;

    // J_base.topLeftCorner(6,3) = state.J.topLeftCorner(6,3);
    
    // J_base * qdot = 0
    constraint_.setMatrix(J_base);
    constraint_.setVector(vel_base);

    return constraint_;
}