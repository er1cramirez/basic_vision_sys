#include "UAVStateMachine.h"

// Constructor
UAVStateMachine::UAVStateMachine(LatestData<EKFStateResult>& stateEstimate,
                               LatestData<ControlOutput>& controlOutput)
    : stateEstimate(stateEstimate),
      controlOutput(controlOutput),
      currentState(UAVState::IDLE),
      currentMode(ControlMode::POSITION_HOLD){
    
    // Initialize controllers
    // positionController = std::make_unique<PositionController>();
    velocityController = std::make_unique<VelocityController>();

    
    // Initialize targets
    positionTarget = Eigen::Vector3d::Zero();
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
    // if (!positionController->initialize()) {
    //     return false;
    // }
    if (!velocityController->initialize()) {
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
        UAV::logger().Write("WARN", "TimeUS,Message", "QZ",
                           UAV::logger().getMicroseconds(),
                           "No state estimate available in non-IDLE state");
        
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
        case UAVState::HOVER:
            executeHover();
            break;
    }
    
    // Log current state
    UAV::logger().Write("STAT", "TimeUS,State,Mode", "QII",
                       UAV::logger().getMicroseconds(),
                       static_cast<int>(currentState),
                       static_cast<int>(currentMode));
    
    // If we have state information, compute and output control commands
    if (hasState) {
        ControlOutput control = computeControlOutput(state);
        controlOutput.update(control);
        // Update mavlink control output
        UAV::logger().Write("CTRL", "TimeUS,u_x,u_y,u_z", "Qfff",
                           UAV::logger().getMicroseconds(),
                           static_cast<float>(control.u_desired.x()),
                           static_cast<float>(control.u_desired.y()),
                           static_cast<float>(control.u_desired.z()));
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
    }
}


// State: Hover
void UAVStateMachine::executeHover() {
    // In hover state, we maintain position at the positionTarget
    // The control output is computed in computeControlOutput
}



// Compute control output based on current state and mode
ControlOutput UAVStateMachine::computeControlOutput(const EKFStateResult& state) {
    ControlOutput control;
    
    // Use appropriate controller based on mode
    switch (currentMode) {
        case ControlMode::POSITION_HOLD:
            // control = positionController->computeControl(state, positionTarget);
            control = velocityController->computeControl(state, positionTarget);
            break;
        default:
            // Default to position hold
            // control = positionController->computeControl(state, positionTarget);
            control = velocityController->computeControl(state, positionTarget);
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
    
    UAV::logger().Write("TPOS", "TimeUS,X,Y,Z", "Qfff",
                       UAV::logger().getMicroseconds(),
                       static_cast<float>(position.x()),
                       static_cast<float>(position.y()),
                       static_cast<float>(position.z()));
}