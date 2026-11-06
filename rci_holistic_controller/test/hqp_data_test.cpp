#include "rci_holistic_controller/data/hqp_data.hpp"
#include <ros/ros.h>
#include <limits>
#include <stdexcept>

using chanhqp::data::HQPData;

// Constant tasks isolate the solver from robot kinematics.
class ConstantTask : public TaskBase
{
public:
    ConstantTask(bool equality, const Eigen::MatrixXd &A, const Eigen::VectorXd &b)
        : TaskBase("test")
    {
        if (equality) c_ = std::make_shared<ConstraintEquality>("test");
        else c_ = std::make_shared<ConstraintInequality>("test");
        c_->setMatrix(A);
        c_->setVector(b);
    }
    bool getSE3Traj(pinocchio::SE3) override { return false; }
    bool getVectorTraj(Eigen::VectorXd b) override { return c_->setVector(b); }
    const std::shared_ptr<ConstraintBase> constraint() const override { return c_; }
    std::shared_ptr<ConstraintBase> compute(State) override { return c_; }
private:
    std::shared_ptr<ConstraintBase> c_;
};

class InspectHQP : public HQPData
{
public:
    size_t slackCount() const { return slack_eq_.size(); }
    double inequalitySlack(int level) const { return slack_ineq_.at(level)(0); }
};

void require(bool ok)
{
    if (!ok) throw std::runtime_error("HQP regression failed");
}

std::shared_ptr<ConstantTask> scalarTask(bool eq, double a, double b)
{
    return std::make_shared<ConstantTask>(
        eq, Eigen::MatrixXd::Constant(1, 1, a), Eigen::VectorXd::Constant(1, b));
}

int main()
{
    ros::Time::init();
    // Repeated solves must not retain previous residuals or grow slack storage.
    InspectHQP repeated;
    auto target = scalarTask(true, 1, 1);
    repeated.addTask(target, 1);
    for (int i = 1; i <= 100; ++i)
    {
        target->getVectorTraj(Eigen::VectorXd::Constant(1, i));
        repeated.compute_problem_data(State{});
        require(std::abs(repeated.solve()(0) - i / 1.00001) < 1e-7);
        require(repeated.slackCount() == 2);
    }

    // A conflicting lower equality cannot change the higher level result.
    HQPData equality;
    equality.addTask(scalarTask(true, 1, 1), 1);
    equality.addTask(scalarTask(true, 1, -10), 2);
    equality.compute_problem_data(State{});
    require(std::abs(equality.solve()(0) - 1 / 1.00001) < 1e-7);

    // A soft inequality remains feasible against a conflicting hard equality.
    InspectHQP relaxed;
    relaxed.addTask(scalarTask(true, 1, 0), 0);
    relaxed.addTask(scalarTask(false, 1, -1), 1); // x + s >= 1
    relaxed.addTask(scalarTask(true, 1, 10), 2);
    relaxed.compute_problem_data(State{});
    require(std::abs(relaxed.solve()(0)) < 1e-7);
    require(std::abs(relaxed.inequalitySlack(1) - 1) < 1e-7);

    // Preserve a higher inequality's relaxed bound against a lower equality.
    HQPData inequality;
    inequality.addTask(scalarTask(false, 1, -1), 1);
    inequality.addTask(scalarTask(true, 1, -10), 2);
    inequality.compute_problem_data(State{});
    require(std::abs(inequality.solve()(0) - 1 / 1.00001) < 1e-7);

    // Mixed tasks with different row counts preserve the upper task's free DoF.
    HQPData mixed;
    Eigen::MatrixXd upper(1, 2);
    upper << 1, 0;
    mixed.addTask(std::make_shared<ConstantTask>(
        true, upper, Eigen::VectorXd::Ones(1)), 1);
    Eigen::VectorXd lower(2);
    lower << -10, 2;
    mixed.addTask(std::make_shared<ConstantTask>(
        true, Eigen::MatrixXd::Identity(2, 2), lower), 2);
    mixed.compute_problem_data(State{});
    Eigen::VectorXd result = mixed.solve();
    require(std::abs(result(0) - 1 / 1.00001) < 1e-7);
    require(std::abs(result(1) - 2 / 1.00001) < 1e-7);

    // Infeasible hard constraints return zero, never the solver's invalid x.
    HQPData infeasible;
    infeasible.addTask(scalarTask(false, 1, -1), 0);
    infeasible.addTask(scalarTask(false, -1, 0), 0);
    infeasible.addTask(scalarTask(true, 1, 5), 1);
    infeasible.compute_problem_data(State{});
    require(infeasible.solve().isZero());
    infeasible.initalize_all_data();
    infeasible.addTask(scalarTask(true, 1, 2), 1);
    infeasible.compute_problem_data(State{});
    require(std::abs(infeasible.solve()(0) - 2 / 1.00001) < 1e-7);

    // Invalid numeric input is rejected before entering eiquadprog.
    HQPData invalid;
    invalid.addTask(scalarTask(true, 1, std::numeric_limits<double>::quiet_NaN()), 1);
    invalid.compute_problem_data(State{});
    require(invalid.solve().isZero());

    bool duplicate_rejected = false;
    try { equality.addTask(scalarTask(true, 1, 0), 1); }
    catch (const std::invalid_argument &) { duplicate_rejected = true; }
    require(duplicate_rejected);
}
