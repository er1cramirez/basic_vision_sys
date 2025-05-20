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
    PositionController() : kp(2.0), kd(0.5), maxControl(1.0) {}
    ~PositionController() override = default;
    
    bool initialize() override {
        return true;
    }
    
    void reset() override {
        // Reset controller state
    }
    
    /**
     * @brief Set the controller gains
     * @param proportionalGain Proportional gain (kp)
     * @param derivativeGain Derivative gain (kd)
     */
    void setGains(double proportionalGain, double derivativeGain) {
        kp = proportionalGain;
        kd = derivativeGain;
    }
    
    /**
     * @brief Set maximum control output magnitude
     * @param max Maximum control magnitude
     */
    void setMaxControl(double max) {
        maxControl = max;
    }
    
    /**
     * @brief Compute control output from state estimate
     * @param state Current state estimate
     * @param target Target position
     * @return Control output
     */
    DroneControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& target) {
        DroneControlOutput output;
        
        // PD controller: u = kp(pos_error) + kd(-vel)
        Eigen::Vector3d error = target - state.position;
        
        // PD control calculation
        output.u_desired = error * kp + (-state.velocity) * kd;
        
        // Limit control output magnitude if needed
        if (output.u_desired.norm() > maxControl) {
            output.u_desired = output.u_desired.normalized() * maxControl;
        }
        
        // Set u_desired_dot to zeros as specified
        output.u_desired_dot = Eigen::Vector3d::Zero();
        
        return output;
    }
    
private:
    double kp;  // Proportional gain
    double kd;  // Derivative gain
    double maxControl;  // Maximum control output magnitude
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
     * @param targetVel Target velocity
     * @return Control output
     */
    DroneControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& targetVel) {
        DroneControlOutput output;
        
        // Simple P controller for velocity
        Eigen::Vector3d error = targetVel - state.velocity;
        double kv = 1.0;
        output.u_desired = error * kv;
        output.u_desired_dot = Eigen::Vector3d::Zero(); // No derivative control
        
        return output;
    }
};

#endif // CONTROLLERS_H