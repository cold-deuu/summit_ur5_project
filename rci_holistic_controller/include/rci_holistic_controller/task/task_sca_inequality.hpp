#ifndef __task_sca_equality_hpp__
#define __task_sca_equality_hpp__

#include <rci_holistic_controller/task/task_base.hpp>

namespace chanhqp{
    namespace task{
        class TaskSCAInequality : public TaskBase
        {
            public:
                TaskSCAInequality(const std::string & name);
                
                bool getSE3Traj(pinocchio::SE3 trajSample);
                bool getVectorTraj(Eigen::VectorXd );

                chanhqp::constraint::ConstraintBase & compute(State state);

            protected:
                chanhqp::constraint::ConstraintInequality constraint_;
        };
    };
};

#endif
