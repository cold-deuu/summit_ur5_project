#pragma once

// QP Header
#include "eiquadprog/eiquadprog.hpp"

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

                void qp_initialize();

                std::map<int, ConstraintBase*> get_constraint()
                {
                    return constraint_;
                }

            Eigen::VectorXd solve();

            protected:
                // Pointer??
                std::map<int, std::shared_ptr<TaskBase>> tasks_;
                std::map<int, ConstraintBase*> constraint_;

                std::vector<ConstraintBase*> safety_constraint_;

                // Tasks 
                int num_task_;


                // QP Components
                Eigen::MatrixXd Q;
                Eigen::VectorXd C;
                Eigen::MatrixXd A_eq, A_ineq;
                Eigen::VectorXd b_eq, b_ineq;

                // Eigen::VectorXd slack_eq_, slack_ineq_;
                std::vector<Eigen::VectorXd> slack_eq_, slack_ineq_;
                Eigen::VectorXd sol_;


        };



    };
};