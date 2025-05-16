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
        double kp = 1.0;
        output.targetVelocity = error * kp;
        
        // Limit max velocity
        double maxVel = 1.0;
        if (output.targetVelocity.norm() > maxVel) {
            output.targetVelocity = output.targetVelocity.normalized() * maxVel;
        }
        
        output.targetPosition = target;
        output.positionControlEnabled = true;
        output.velocityControlEnabled = true;
        
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
     * @param targetVel Target velocity
     * @return Control output
     */
    ControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& targetVel) {
        ControlOutput output;
        
        // Simple P controller for velocity
        Eigen::Vector3d error = targetVel - state.velocity;
        double kv = 1.0;
        output.targetAcceleration = error * kv;
        
        // Set target velocity
        output.targetVelocity = targetVel;
        output.velocityControlEnabled = true;
        output.attitudeControlEnabled = true;
        
        return output;
    }
};

/**
 * @class AttitudeController
 * @brief Controller for attitude control
 */
class AttitudeController : public ControllerBase {
public:
    AttitudeController() = default;
    ~AttitudeController() override = default;
    
    bool initialize() override {
        return true;
    }
    
    void reset() override {
        // Reset controller state
    }
    
    /**
     * @brief Compute control output from state estimate
     * @param state Current state estimate
     * @param targetAtt Target attitude (roll, pitch, yaw)
     * @return Control output
     */
    ControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& targetAtt) {
        ControlOutput output;
        
        // Set target attitude
        output.targetAttitudeRPY = targetAtt;
        output.attitudeControlEnabled = true;
        
        return output;
    }
};

#endif // CONTROLLERS_H