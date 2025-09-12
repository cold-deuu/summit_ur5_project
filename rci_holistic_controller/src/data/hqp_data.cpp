#include "rci_holistic_controller/data/hqp_data.hpp"

using namespace chanhqp::data;

HQPData::HQPData(){}

void HQPData::addTask(std::shared_ptr<TaskBase> task, int priority)
{
    auto [it, inserted] = tasks_.emplace(priority, task);
    assert(inserted && "Duplicate priority!");
}

void HQPData::compute_problem_data(State state)
{
    constraint_.clear(); // map<int, ConstraintBase*>

    for (const auto& [priority, task_ptr] : tasks_) {
        // priority 중복이 있으면 에러로 보고 싶다면:
        auto [it, inserted] = constraint_.emplace(priority, &task_ptr->compute(state));
        assert(inserted && "Duplicate priority!");
    }

    assert(!constraint_.empty());
    auto max_priority = constraint_.rbegin()->first;
    num_task_ = static_cast<int>(constraint_.size());

    // priority가 1..N 연속이라는 전제 검증(선택)
    std::cout<<"Max :"<<max_priority<<"Num Task :"<<num_task_<<std::endl;
    assert(max_priority == num_task_);

}

// Eigen::VectorXd HQPData::solve()
// {

// };