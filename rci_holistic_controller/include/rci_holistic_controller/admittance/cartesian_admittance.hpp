#pragma once
#include <Eigen/Dense>
#include <pinocchio/spatial/explog.hpp>
#include <cmath>
#include <limits>

namespace holistic_controller { namespace admittance {
using Vector6 = Eigen::Matrix<double, 6, 1>;

// Shared implementation extracted from SummitUR5Wrapper's existing M-D-K loop.
// Fixed-size storage: configure outside the control loop; step never allocates.
class CartesianAdmittance {
public:
    struct Parameters {
        Vector6 mass = Vector6::Ones();
        Vector6 damping = Vector6::Ones();
        Vector6 stiffness = Vector6::Zero();
        Vector6 deadband = Vector6::Zero();
        Vector6 displacement_limit = Vector6::Constant(std::numeric_limits<double>::infinity());
        Vector6 velocity_limit = Vector6::Constant(std::numeric_limits<double>::infinity());
        Vector6 acceleration_limit = Vector6::Constant(std::numeric_limits<double>::infinity());
        double linear_velocity_norm_limit = std::numeric_limits<double>::infinity();
        double angular_velocity_norm_limit = std::numeric_limits<double>::infinity();
        double cutoff_hz = 0;
    };
    Parameters parameters;
    Vector6 displacement = Vector6::Zero(), velocity = Vector6::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

    void reset() {
        displacement.setZero(); velocity.setZero(); filtered_.setZero();
        rotation.setIdentity();
    }
    void filter(const Vector6 &bias_corrected_sensor, double dt, Vector6 &out) {
        const double alpha = parameters.cutoff_hz > 0 ?
            1.0 - std::exp(-2.0 * 3.141592653589793 * parameters.cutoff_hz * dt) : 1.0;
        filtered_ += alpha * (bias_corrected_sensor - filtered_);
        for (int i=0; i<6; ++i) {
            const double value = filtered_(i);
            out(i) = std::copysign(std::max(0.0, std::abs(value)-parameters.deadband(i)), value);
        }
    }
    void step(const Vector6 &world_wrench, double dt) {
        Vector6 acceleration = (world_wrench - parameters.damping.cwiseProduct(velocity) -
            parameters.stiffness.cwiseProduct(displacement)).cwiseQuotient(parameters.mass);
        for(int i=0;i<6;++i) acceleration(i)=std::max(-parameters.acceleration_limit(i),
            std::min(parameters.acceleration_limit(i),acceleration(i)));
        velocity += acceleration * dt;
        for (int i=0; i<6; ++i)
            velocity(i) = std::max(-parameters.velocity_limit(i),
                                  std::min(parameters.velocity_limit(i), velocity(i)));
        const double linear_norm=velocity.head<3>().norm(), angular_norm=velocity.tail<3>().norm();
        if(linear_norm>parameters.linear_velocity_norm_limit) velocity.head<3>()*=parameters.linear_velocity_norm_limit/linear_norm;
        if(angular_norm>parameters.angular_velocity_norm_limit) velocity.tail<3>()*=parameters.angular_velocity_norm_limit/angular_norm;
        displacement.head<3>() += velocity.head<3>() * dt;
        rotation = pinocchio::exp3(Eigen::Vector3d(velocity.tail<3>() * dt)) * rotation;
        displacement.tail<3>() = pinocchio::log3(rotation);
        bool clipped_rotation = false;
        for (int i=0; i<6; ++i) {
            const double bound = parameters.displacement_limit(i);
            if (std::abs(displacement(i)) > bound) {
                displacement(i) = std::copysign(bound, displacement(i));
                if (displacement(i)*velocity(i)>0) velocity(i)=0; // anti-windup
                clipped_rotation |= i>=3;
            }
        }
        if (clipped_rotation) rotation = pinocchio::exp3(Eigen::Vector3d(displacement.tail<3>()));
    }
private:
    Vector6 filtered_ = Vector6::Zero();
};
}}
