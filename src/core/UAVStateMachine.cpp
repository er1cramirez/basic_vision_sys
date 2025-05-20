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
    
    // Create and configure controllers
    positionController = std::make_unique<PositionController>();
    velocityController = std::make_unique<VelocityController>();
    
    // Set PD controller gains (these will need tuning)
    dynamic_cast<PositionController*>(positionController.get())->setGains(0.1, 0.05);
    dynamic_cast<PositionController*>(positionController.get())->setMaxControl(0.4);
    
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
    
    // Add timeout handling for safety
    if (!hasState && currentState != UAVState::IDLE && currentState != UAVState::EMERGENCY) {
        int64_t timeSinceUpdate = stateEstimate.timeSinceUpdate();
        if (timeSinceUpdate > 500) {  // 500ms timeout
            setState(UAVState::EMERGENCY);
            UAV::logger().Write("EMRG", "TimeUS,Reason", "QZ",
                              UAV::logger().getMicroseconds(),
                              "State estimate timeout");
        }
    }
    
    // Execute appropriate state behavior
    switch (currentState) {
        case UAVState::IDLE:
            executeIdle();
            break;
        case UAVState::INITIALIZING:
            executeInitializing();
            break;
        case UAVState::HOVER:  // Primary position control state
            executeHover();
            break;
        case UAVState::LANDING:
            executeLanding();
            break;
        case UAVState::EMERGENCY:
            executeEmergency();
            break;
        default:
            break;
    }
    
    // If we have state information, compute and output control commands
    if (hasState && currentState != UAVState::IDLE) {
        DroneControlOutput control = computeControlOutput(state);
        controlOutput.update(control);
        
        // Log control output periodically
        static int logCounter = 0;
        if (++logCounter % 10 == 0) {  // Log every 10th control cycle
            UAV::logger().Write("CTRL", "TimeUS,PosX,PosY,PosZ,TgtX,TgtY,TgtZ,UX,UY,UZ", "Qfffffffffff",
                              UAV::logger().getMicroseconds(),
                              static_cast<float>(state.position.x()),
                              static_cast<float>(state.position.y()),
                              static_cast<float>(state.position.z()),
                              static_cast<float>(positionTarget.x()),
                              static_cast<float>(positionTarget.y()),
                              static_cast<float>(positionTarget.z()),
                              static_cast<float>(control.u_desired.x()),
                              static_cast<float>(control.u_desired.y()),
                              static_cast<float>(control.u_desired.z()));
        }
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
    EKFStateResult state;
    if (stateEstimate.getData(state)) {
        // Check if we're close to target
        static bool positionReachedLogged = false;
        if (isPositionReached(state.position, positionTarget)) {
            positionReachedLogged = false;
            if (!positionReachedLogged) {
                UAV::logger().Write("POSN", "TimeUS,Status", "QZ",
                                  UAV::logger().getMicroseconds(),
                                  "Position target reached");
                positionReachedLogged = true;
            }
        } else {
            positionReachedLogged = false;
        }
    }
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