#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "ControlTypes.h"
#include "aruco_ekf_estimator.h"
#include <Eigen/Dense>

/**
 * @class ControllerBase
 * @brief Base class for all controllers
 */
class ControllerBase {
public:
    ControllerBase() = default;
    virtual ~ControllerBase() = default;
    
    /**
     * @brief Initialize the controller
     * @return True if initialization was successful
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief Reset the controller
     */
    virtual void reset() = 0;
};

/**
 * @class PositionController
 * @brief Controller for position control
 */
class PositionController : public ControllerBase {
public:
    PositionController() = default;
    ~PositionController() override = default;
    
    bool initialize() override {
        return true;
    }
    
    void reset() override {
        // Reset controller state
    }
    
    /**
     * @brief Compute control output from state estimate
     * @param state Current state estimate
     * @param target Target position
     * @return Control output
     */
    ControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& target) {
        ControlOutput output;
        
        // Simple P controller for position
        Eigen::Vector3d error = target - state.position;
        double kp = 0.02;
        double kd = 0.035;
        output.u_desired = error * kp + (-state.velocity * kd);
        
        output.u_desired_dot = Eigen::Vector3d::Zero(); // No acceleration control
        
        return output;
    }
};

/**
 * @class VelocityController
 * @brief Controller for velocity control
 */
class VelocityController : public ControllerBase {
public:
    VelocityController() = default;
    ~VelocityController() override = default;
    
    bool initialize() override {
        return true;
    }
    
    void reset() override {
        // Reset controller state
    }
    
    /**
     * @brief Compute control output from state estimate
     * @param state Current state estimate
     * @param target Target position
     * @return Control output
     */
    ControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& target) {
        ControlOutput output;

        double cr = 0.1; // Radial velocity gain
        double ct = 0.0; // Tangential velocity gain

        // Simplified version of planning equations(No height dependence)
        // Vectorial distance to target
        Eigen::Vector3d d_vector = -state.position;
        // discard the z component
        d_vector.z() = 0;
        Eigen::Vector3d d_vector_dot = -state.velocity;
        // discard the z component
        d_vector_dot.z() = 0;

        // Unit vector to target
        Eigen::Vector3d _R = d_vector.normalized();
        // Scalar distance to target
        double d = d_vector.norm();

        // Derivative of d respect to time
        // d/dt d = R' * d_vector_dot
        // where R' is the transpose of R
        double d_dot = _R.dot(d_vector_dot);

        // Then for R_dot we have:
        // R_dot = (I - R * R') * d_vector_dot / d_vector.norm()
        // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        // Eigen::Matrix3d R_RT = _R * _R.transpose();
        // Eigen::Vector3d R_dot = (I - R_RT) * d_vector_dot / d;

        // Alternative (more efficient) computation:
        // R_dot = (d_vector_dot - _R * (_R.dot(d_vector_dot))) / d
        // Which is equivalent to:
        Eigen::Vector3d R_dot = (d_vector_dot - _R * d_dot) / d;
        

        // Tangential unity vector (z axis)
        Eigen::Vector3d _T = Eigen::Vector3d(0, 0, 1);

        // Radial velocity regulator mu_r = tanh(d)
        double mu_r = std::tanh(d);
        // Tangential velocity regulator mu_t = sech(d)
        double mu_t = 1 / std::cosh(d);

        // Calculate derivatives of the velocity regulators
        // d/dt(mu_r) = d/dt(tanh(d)) = sech²(d) * d_dot
        double mu_r_dot = (1 - mu_r * mu_r) * d_dot;  // since sech²(d) = 1 - tanh²(d)

        // d/dt(mu_t) = d/dt(sech(d)) = -sech(d) * tanh(d) * d_dot
        double mu_t_dot = -mu_t * mu_r * d_dot;


        // Desired velocity vector
        Eigen::Vector3d v_desired = mu_r * cr * _R + mu_t * ct * _T;

        // Calculate v_desired_dot
        // d/dt(v_desired) = d/dt(mu_r * cr * _R + mu_t * ct * _T)
        //                 = mu_r_dot * cr * _R + mu_r * cr * R_dot + mu_t_dot * ct * _T
        Eigen::Vector3d v_desired_dot = mu_r_dot * cr * _R + 
                                        mu_r * cr * R_dot + 
                                        mu_t_dot * ct * _T;
        // Note: _T is constant, so its derivative is zero
        // Calculate velocity error and its derivative
        Eigen::Vector3d error = state.velocity - v_desired;
        Eigen::Vector3d error_dot = state.acceleration - v_desired_dot;  // assuming you have acceleration in state

        // Control and its derivative
        double kp = -0.02;
        output.u_desired = error * kp;
        output.u_desired_dot = error_dot * 0.0;
        
        return output;
    }
};

#endif // CONTROLLERS_H