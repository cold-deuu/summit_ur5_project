#ifndef __task_se3_equality_hpp__
#define __task_se3_equality_hpp__

#include <rci_holistic_controller/task/task_base.hpp>
#include <rci_holistic_controller/constraint/constraint_equality.hpp>

namespace chanhqp{
    namespace task{
        class TaskSE3Equality : public TaskBase
        {
            public:
                TaskSE3Equality(const std::string & name);
                
                const bool & getSE3Traj(pinocchio::SE3 trajSample) const;
                const bool & getVectorTraj(Eigen::VectorXd ) const;

                chanhqp::constraint::ConstraintBase & compute(State state);

            protected:
                chanhqp::constraint::ConstraintEquality constraint_;
                pinocchio::SE3 ref_se3_;
        }
    }
}

#endif
