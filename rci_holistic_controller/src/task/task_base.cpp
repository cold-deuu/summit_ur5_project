#include <rci_holistic_controller/task/task_base.hpp>

using namespace chanhqp::task;

TaskBase::TaskBase(std::string & name):
    name_(name)

const double TaskBase::Kp(double Kp) const
{
    Kp_ = Kp;
}

const double TaskBase::Kd(double Kd) const
{
    Kd_ = Kd;
}
