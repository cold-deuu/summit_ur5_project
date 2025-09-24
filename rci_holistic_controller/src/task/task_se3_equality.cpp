#include <rci_holistic_controller/task/task_se3_equality.hpp>

using namespace chanhqp::constraint;
using namespace chanhqp::task;

TaskSE3Equality::TaskSE3Equality(const std::string & name, bool is_admittance):
    TaskBase(name), is_admittance_(is_admittance), constraint_(std::make_shared<ConstraintEquality>(name)){
        task_space_admittance_.setZero(6);
    };

bool TaskSE3Equality::getSE3Traj(pinocchio::SE3 trajSample) {
    ref_se3_ = trajSample;
    return true;
}

bool TaskSE3Equality::getVectorTraj(Eigen::VectorXd ) {
    assert(false);
    return false;
}

std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskSE3Equality::compute(State state)
{
    pinocchio::SE3 dMi = state.oMi.inverse() * ref_se3_;
    pinocchio::Motion x_err_motion = pinocchio::log6(dMi);
    Eigen::VectorXd x_err = x_err_motion.toVector();
    Eigen::VectorXd des_xdot = Kp_ * x_err;

    if (is_admittance_)
    {
        des_xdot += task_space_admittance_;
        std::cout<<"[TaskSE3Equality] task_space_admittance_ : "<<task_space_admittance_.transpose()<<std::endl;
    }

    std::cout<<"[TaskSE3Equality] task_space_admittance_ : "<<task_space_admittance_.transpose()<<std::endl;

    constraint_ -> setMatrix(state.J);
    constraint_ -> setVector(des_xdot);
    return constraint_;
}

// ConstraintBase & TaskSE3Equality::compute(State state)
// {
//     pinocchio::SE3 dMi = state.oMi.inverse() * ref_se3_;
//     pinocchio::Motion x_err_motion = pinocchio::log6(dMi);
//     Eigen::VectorXd x_err = x_err_motion.toVector();
//     Eigen::VectorXd des_xdot = Kp_ * x_err;
//     constraint_.setMatrix(state.J);
//     constraint_ .setVector(x_err);
//     return constraint_;
// }


const std::shared_ptr<chanhqp::constraint::ConstraintBase> TaskSE3Equality::constraint() const
{
  return constraint_;
}
