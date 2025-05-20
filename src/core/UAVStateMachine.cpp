#include "UAVStateMachine.h"

// Constructor
UAVStateMachine::UAVStateMachine(LatestData<EKFStateResult>& stateEstimate,
                               LatestData<DroneControlOutput>& controlOutput)
    : stateEstimate(stateEstimate),
      controlOutput(controlOutput),
      currentState(UAVState::IDLE),
      currentMode(ControlMode::MANUAL) {
    
    // Initialize controllers
    positionController = std::make_unique<PositionController>();
    velocityController = std::make_unique<VelocityController>();
    
    // Initialize targets
    positionTarget = Eigen::Vector3d::Zero();
    velocityTarget = Eigen::Vector3d::Zero();
}

// Destructor
UAVStateMachine::~UAVStateMachine() {
    // The unique_ptrs will clean up the controllers
}

// Initialize the state machine
bool UAVStateMachine::initialize() {
    // Log initialization
    UAV::logger().Write("INIT", "TimeUS,Component", "QZ",
                       UAV::logger().getMicroseconds(),
                       "UAVStateMachine");
    
    // Initialize controllers
    if (!positionController->initialize() ||
        !velocityController->initialize()) {
        return false;
    }
    
    return true;
}

// Execute one cycle of the state machine
void UAVStateMachine::execute() {
    // Get latest state estimate
    EKFStateResult state;
    bool hasState = stateEstimate.getData(state);
    
    if (!hasState && currentState != UAVState::IDLE) {
        // No state information available, but we're supposed to be operating
        // Log warning and consider emergency action
        
        // If state has been unavailable for too long, we might want to go to emergency mode
        // This would require tracking how long we've been without a state update
    }
    
    // Execute appropriate state behavior
    switch (currentState) {
        case UAVState::IDLE:
            executeIdle();
            break;
        case UAVState::INITIALIZING:
            executeInitializing();
            break;
        case UAVState::TAKEOFF:
            executeTakeoff();
            break;
        case UAVState::HOVER:
            executeHover();
            break;
        case UAVState::LANDING:
            executeLanding();
            break;
        case UAVState::EMERGENCY:
            executeEmergency();
            break;
    }
    
    // If we have state information, compute and output control commands
    if (hasState) {
        DroneControlOutput control = computeControlOutput(state);
        controlOutput.update(control);
    }
}

// State: Idle
void UAVStateMachine::executeIdle() {
    // In idle, we don't output control commands
    // We just wait for a command to transition to another state
}

// State: Initializing
void UAVStateMachine::executeInitializing() {
    // Check if we have a valid state estimate
    EKFStateResult state;
    bool hasState = stateEstimate.getData(state);
    
    if (hasState && state.valid) {
        // We have a valid state, transition to hover
        setState(UAVState::HOVER);
        
        // Use current position as hover target
        positionTarget = state.position;
        
        UAV::logger().Write("TRAN", "TimeUS,FromState,ToState", "QII",
                           UAV::logger().getMicroseconds(),
                           static_cast<int>(UAVState::INITIALIZING),
                           static_cast<int>(UAVState::HOVER));
    }
}

// State: Takeoff
void UAVStateMachine::executeTakeoff() {
    // Check if we have a valid state estimate
    
}

// State: Hover
void UAVStateMachine::executeHover() {
    // In hover state, we maintain position at the positionTarget
    // The control output is computed in computeControlOutput
}
// State: Landing
void UAVStateMachine::executeLanding() {

}

// State: Emergency
void UAVStateMachine::executeEmergency() {

}

// Compute control output based on current state and mode
DroneControlOutput UAVStateMachine::computeControlOutput(const EKFStateResult& state) {
    DroneControlOutput control;
    
    // Use appropriate controller based on mode
    switch (currentMode) {
        case ControlMode::POSITION_CONTROL:
            control = positionController->computeControl(state, positionTarget);
            break;
            
        case ControlMode::VELOCITY_CONTROL:
            control = velocityController->computeControl(state, velocityTarget);
            break;

        default:
            // Default to position hold
            control = positionController->computeControl(state, positionTarget);
            break;
    }
    return control;
}

// Set the UAV state
void UAVStateMachine::setState(UAVState newState) {
    if (newState != currentState) {
        UAVState oldState = currentState;
        currentState = newState;
        
        UAV::logger().Write("STCH", "TimeUS,OldState,NewState", "QII",
                           UAV::logger().getMicroseconds(),
                           static_cast<int>(oldState),
                           static_cast<int>(newState));
        
        // Reset state-specific variables
        switch (newState) {
            case UAVState::WAYPOINT_NAVIGATION:
                // currentWaypointIndex = 0;
                // waypointReached = false;
                break;
                
            default:
                break;
        }
    }
}

// Set the control mode
void UAVStateMachine::setControlMode(ControlMode newMode) {
    if (newMode != currentMode) {
        ControlMode oldMode = currentMode;
        currentMode = newMode;
        
        UAV::logger().Write("MDCH", "TimeUS,OldMode,NewMode", "QII",
                           UAV::logger().getMicroseconds(),
                           static_cast<int>(oldMode),
                           static_cast<int>(newMode));
        
        // Reset mode-specific variables
        switch (newMode) {
            case ControlMode::VELOCITY_CONTROL:
                velocityTarget = Eigen::Vector3d::Zero();
                break;
                
            default:
                break;
        }
    }
}

// Get current UAV state
UAVState UAVStateMachine::getState() const {
    return currentState;
}

// Get current control mode
ControlMode UAVStateMachine::getControlMode() const {
    return currentMode;
}

// Set position target
void UAVStateMachine::setPositionTarget(const Eigen::Vector3d& position) {
    positionTarget = position;
}

// Set velocity target
void UAVStateMachine::setVelocityTarget(const Eigen::Vector3d& velocity) {
    velocityTarget = velocity;
    
}


// Check if position has been reached
bool UAVStateMachine::isPositionReached(const Eigen::Vector3d& current, 
                                      const Eigen::Vector3d& target,
                                      double threshold) {
    return (current - target).norm() < threshold;
}