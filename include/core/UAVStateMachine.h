#ifndef UAV_STATE_MACHINE_H
#define UAV_STATE_MACHINE_H

#include "ControlTypes.h"
#include "LatestData.h"
#include "aruco_ekf_estimator.h"
#include "Logger.h"
#include "Controllers.h"
#include <memory>

/**
 * @enum UAVState
 * @brief States for the UAV state machine
 */
enum class UAVState {
    IDLE,
    INITIALIZING,
    HOVER
};

/**
 * @enum ControlMode
 * @brief Control modes for the UAV
 */
enum class ControlMode {
    MANUAL,
    POSITION_HOLD
};

/**
 * @class UAVStateMachine
 * @brief State machine for UAV control
 */
class UAVStateMachine {
public:
    /**
     * @brief Constructor
     * @param stateEstimate Reference to EKF state data
     * @param controlOutput Reference to control output data
     */
    UAVStateMachine(LatestData<EKFStateResult>& stateEstimate,
                   LatestData<ControlOutput>& controlOutput);
    
    /**
     * @brief Destructor
     */
    ~UAVStateMachine();
    
    /**
     * @brief Initialize the state machine
     * @return True if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Execute one cycle of the state machine
     */
    void execute();
    
    /**
     * @brief Set the UAV state
     * @param newState New state to set
     */
    void setState(UAVState newState);
    
    /**
     * @brief Set the control mode
     * @param newMode New control mode to set
     */
    void setControlMode(ControlMode newMode);
    
    /**
     * @brief Get current UAV state
     * @return Current state
     */
    UAVState getState() const;
    
    /**
     * @brief Get current control mode
     * @return Current control mode
     */
    ControlMode getControlMode() const;
    
    /**
     * @brief Set position target for position control
     * @param position Target position
     */
    void setPositionTarget(const Eigen::Vector3d& position);
    
    
private:
    // State and mode
    UAVState currentState;
    ControlMode currentMode;
    
    // Shared data references
    LatestData<EKFStateResult>& stateEstimate;
    LatestData<ControlOutput>& controlOutput;
    
    // Controllers
    std::unique_ptr<PositionController> positionController;
    
    // Targets
    Eigen::Vector3d positionTarget;
    
    // State execution methods
    void executeIdle();
    void executeInitializing();
    void executeHover();
    
    // Control methods
    ControlOutput computeControlOutput(const EKFStateResult& state);
};

#endif // UAV_STATE_MACHINE_H