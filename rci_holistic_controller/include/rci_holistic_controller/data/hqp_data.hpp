#pragma once

// HQP Solver : Math
#include "rci_holistic_controller/math/math.hpp"

// HQP Solver : Constraint
#include "rci_holistic_controller/constraint/constraint_base.hpp"
#include "rci_holistic_controller/constraint/constraint_equality.hpp"
#include "rci_holistic_controller/constraint/constraint_inequality.hpp"

// HQP Solver : Task
#include "rci_holistic_controller/task/tasks.hpp"

using namespace chanhqp::task;
using namespace chanhqp::constraint;

namespace chanhqp{
    namespace data{
        class HQPData
        {
            public:
                HQPData();
                void addTask(std::shared_ptr<TaskBase> task, int priority);
                void compute_problem_data(State state);

                std::map<int, ConstraintBase*> get_constraint()
                {
                    return constraint_;
                }

            // Eigen::VectorXd solve();

            protected:
                // Pointer??
                std::map<int, std::shared_ptr<TaskBase>> tasks_;
                std::map<int, ConstraintBase*> constraint_;

                // Tasks 
                int num_task_;



        };



    };
};