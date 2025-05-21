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

        // Vectorial distance to target
        Eigen::Vector3d d_vector = -state.position;
        // discard the z component
        d_vector.z() = 0;
        // Unit vector to target
        Eigen::Vector3d _R = d_vector.normalized();
        // Escalar distance to target
        double d = d_vector.norm();
        // Tangential unity vector (z axis)
        Eigen::Vector3d _T = Eigen::Vector3d(0, 0, 1);

        // Radial velocity regulator mu_r = tanh(d)
        double mu_r = std::tanh(d);
        // Tangential velocity regulator mu_t = sech(d)
        double mu_t = 1 / std::cosh(d);

        // Desired velocity vector
        Eigen::Vector3d v_desired = mu_r * cr * _R + mu_t * ct * _T;
        // Simple P controller for position
        Eigen::Vector3d error = state.velocity - v_desired;
        double kp = -0.02;
        // double kd = 0.035;
        output.u_desired = error * kp;
        
        output.u_desired_dot = Eigen::Vector3d::Zero(); // No acceleration control
        
        return output;
    }
};

#endif // CONTROLLERS_H