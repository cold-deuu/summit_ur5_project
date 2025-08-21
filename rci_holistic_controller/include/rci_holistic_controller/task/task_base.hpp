#ifndef __task_base_hpp__
#define __task_base_hpp__
// Pinocchio Header
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>

// Eigen Header
#include <Eigen/Dense>
#include <Eigen/QR>

// CPP Header
#include <string>

// CHAN HQP CONTROLLER 
#include <rci_holistic_controller/constraint/constraint_base.hpp>
#include <rci_holistic_controller/math/math.hpp>

namespace chanhqp{
    namespace task{
        class TaskBase{
            public:
                TaskBase(const std::string & name);
                
                // Reference
                virtual const bool & getSE3Traj(pinocchio::SE3 trajSample) const = 0;
                virtual const bool & getVectorTraj(Eigen::VectorXd trajSample) const = 0;

                // Gain 
                virtual const double Kp(double Kp) const;
                virtual const double Kd(double Kd) const;

                // Compute
                virtual chanhqp::constraint::ConstraintBase & compute(State state) = 0;

                // // Control
                // virtual bool isPositionControl() const = 0;
                // virtual bool isTorqueControl() const = 0;
            
            
            protected:
                std::string name_;
                double Kp_, Kd_;

            }
    }
}


#endif
