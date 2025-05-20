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
    VRPN_HOLD,
    GPS_HOLD,
    VISION_HOLD,
    INITIALIZING,
    TAKEOFF,
    HOVER,
    WAYPOINT_NAVIGATION,
    LANDING,
    EMERGENCY
};

/**
 * @enum ControlMode
 * @brief Control modes for the UAV
 */
enum class ControlMode {
    MANUAL,
    POSITION_CONTROL,
    VELOCITY_CONTROL
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
                   LatestData<DroneControlOutput>& controlOutput);
    
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
    
    /**
     * @brief Set velocity target for velocity control
     * @param velocity Target velocity
     */
    void setVelocityTarget(const Eigen::Vector3d& velocity);
    
    
private:
    // State and mode
    UAVState currentState;
    ControlMode currentMode;
    
    // Shared data references
    LatestData<EKFStateResult>& stateEstimate;
    LatestData<DroneControlOutput>& controlOutput;
    
    // Controllers
    std::unique_ptr<PositionController> positionController;
    std::unique_ptr<VelocityController> velocityController;
    
    // Targets
    Eigen::Vector3d positionTarget;
    Eigen::Vector3d velocityTarget;
    
    // State execution methods
    void executeIdle();
    void executeInitializing();
    void executeTakeoff();
    void executeHover();
    void executeWaypointNavigation();
    void executeLanding();
    void executeEmergency();
    
    // Control methods
    DroneControlOutput computeControlOutput(const EKFStateResult& state);
    bool isPositionReached(const Eigen::Vector3d& current, const Eigen::Vector3d& target, 
                          double threshold = 0.2);
};

#endif // UAV_STATE_MACHINE_H