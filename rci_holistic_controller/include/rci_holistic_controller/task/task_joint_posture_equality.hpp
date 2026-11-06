#ifndef __task_joint_posture_equality_hpp__
#define __task_joint_posture_equality_hpp__

#include <rci_holistic_controller/task/task_base.hpp>

namespace chanhqp{
    namespace task{
        class TaskJointPostureEquality : public TaskBase
        {
            public:
                TaskJointPostureEquality(const std::string & name);
                
                bool getSE3Traj(pinocchio::SE3 );
                bool getVectorTraj(Eigen::VectorXd trajSample);

                // chanhqp::constraint::ConstraintBase & compute(State state);
                std::shared_ptr<chanhqp::constraint::ConstraintBase> compute(State state);
                const std::shared_ptr<chanhqp::constraint::ConstraintBase> constraint() const;

                using Vector6 = Eigen::Matrix<double, 6, 1>;
                Vector6 velocityReference(const Vector6 &qd, const Vector6 &vd,
                                          const Vector6 &ad, const Vector6 &q, double dt) const
                {
                    // Velocity HQP: acceleration enters through midpoint feedforward.
                    return vd + 0.5 * dt * ad + Kp_ * (qd - q);
                }

            protected:
                std::shared_ptr<chanhqp::constraint::ConstraintEquality> constraint_;
                // chanhqp::constraint::ConstraintEquality constraint_;
                Eigen::VectorXd ref_joint_;
        };
    };
};

#endif
