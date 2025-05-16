#include "UAVStateMachine.h"

// Constructor
UAVStateMachine::UAVStateMachine(LatestData<EKFStateResult>& stateEstimate,
                               LatestData<ControlOutput>& controlOutput)
    : stateEstimate(stateEstimate),
      controlOutput(controlOutput),
      currentState(UAVState::IDLE),
      currentMode(ControlMode::POSITION_HOLD),
      currentWaypointIndex(0),
      waypointReached(false) {
    
    // Initialize controllers
    positionController = std::make_unique<PositionController>();
    velocityController = std::make_unique<VelocityController>();
    attitudeController = std::make_unique<AttitudeController>();
    
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
        !velocityController->initialize() ||
        !attitudeController->initialize()) {
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
        case UAVState::TAKEOFF:
            executeTakeoff();
            break;
        case UAVState::HOVER:
            executeHover();
            break;
        case UAVState::WAYPOINT_NAVIGATION:
            executeWaypointNavigation();
            break;
        case UAVState::LANDING:
            executeLanding();
            break;
        case UAVState::EMERGENCY:
            executeEmergency();
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
        
        // Log control output
        UAV::logger().Write("CTRL", "TimeUS,PosX,PosY,PosZ,Thrust", "Qffff",
                           UAV::logger().getMicroseconds(),
                           static_cast<float>(control.targetPosition.x()),
                           static_cast<float>(control.targetPosition.y()),
                           static_cast<float>(control.targetPosition.z()),
                           control.targetThrust);
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
    EKFStateResult state;
    bool hasState = stateEstimate.getData(state);
    
    if (hasState && state.valid) {
        // Check if we've reached takeoff height
        if (isPositionReached(state.position, positionTarget, 0.2)) {
            // Transition to hover state
            setState(UAVState::HOVER);
            
            UAV::logger().Write("TRAN", "TimeUS,FromState,ToState", "QII",
                               UAV::logger().getMicroseconds(),
                               static_cast<int>(UAVState::TAKEOFF),
                               static_cast<int>(UAVState::HOVER));
        }
    }
}

// State: Hover
void UAVStateMachine::executeHover() {
    // In hover state, we maintain position at the positionTarget
    // The control output is computed in computeControlOutput
}

// State: Waypoint Navigation
void UAVStateMachine::executeWaypointNavigation() {
    // Check if we have a valid state estimate
    EKFStateResult state;
    bool hasState = stateEstimate.getData(state);
    
    if (!hasState || !state.valid) {
        return;
    }
    
    // Check if we have any waypoints
    if (waypoints.empty()) {
        // No waypoints, go to hover
        setState(UAVState::HOVER);
        return;
    }
    
    // Get current waypoint
    const Waypoint& currentWaypoint = waypoints[currentWaypointIndex];
    
    // Update position target to current waypoint
    positionTarget = currentWaypoint.position;
    
    // Check if we've reached the waypoint
    if (isPositionReached(state.position, positionTarget, 0.2)) {
        if (!waypointReached) {
            // Just reached the waypoint
            waypointReached = true;
            
            UAV::logger().Write("WPNT", "TimeUS,WaypointIndex", "QI",
                               UAV::logger().getMicroseconds(),
                               currentWaypointIndex);
            
            // If we have a dwell time, we'll wait
            // In a real implementation, we'd track the time we've been at this waypoint
        } else {
            // We've been at this waypoint, move to next
            currentWaypointIndex++;
            waypointReached = false;
            
            // Check if we've completed all waypoints
            if (currentWaypointIndex >= waypoints.size()) {
                // Done with all waypoints, go to hover
                setState(UAVState::HOVER);
                
                UAV::logger().Write("MSND", "TimeUS,Status", "QZ",
                                   UAV::logger().getMicroseconds(),
                                   "Mission completed");
            }
        }
    }
}

// State: Landing
void UAVStateMachine::executeLanding() {
    // Check if we have a valid state estimate
    EKFStateResult state;
    bool hasState = stateEstimate.getData(state);
    
    if (hasState && state.valid) {
        // Update position target to current XY but decrease Z
        positionTarget.x() = state.position.x();
        positionTarget.y() = state.position.y();
        
        // Check if we're close to the ground
        if (state.position.z() < 0.1) {
            // We've landed, go to idle
            setState(UAVState::IDLE);
            
            UAV::logger().Write("TRAN", "TimeUS,FromState,ToState", "QII",
                               UAV::logger().getMicroseconds(),
                               static_cast<int>(UAVState::LANDING),
                               static_cast<int>(UAVState::IDLE));
        }
    }
}

// State: Emergency
void UAVStateMachine::executeEmergency() {
    // In emergency state, we attempt to land immediately
    // For now, this is similar to landing but might be more aggressive
    
    // Check if we have a valid state estimate
    EKFStateResult state;
    bool hasState = stateEstimate.getData(state);
    
    if (hasState && state.valid) {
        // Set position target to current XY but with Z=0
        positionTarget.x() = state.position.x();
        positionTarget.y() = state.position.y();
        positionTarget.z() = 0.0;
    } else {
        // No state information, just set Z to 0
        positionTarget.z() = 0.0;
    }
}

// Compute control output based on current state and mode
ControlOutput UAVStateMachine::computeControlOutput(const EKFStateResult& state) {
    ControlOutput control;
    
    // Use appropriate controller based on mode
    switch (currentMode) {
        case ControlMode::POSITION_HOLD:
            control = positionController->computeControl(state, positionTarget);
            break;
            
        case ControlMode::VELOCITY_CONTROL:
            control = velocityController->computeControl(state, velocityTarget);
            break;
            
        case ControlMode::ATTITUDE_CONTROL:
            // For now, just use a fixed attitude
            control = attitudeController->computeControl(state, Eigen::Vector3d::Zero());
            break;
            
        case ControlMode::WAYPOINT_FOLLOWING:
            // Similar to position hold but with waypoint as target
            control = positionController->computeControl(state, positionTarget);
            break;
            
        default:
            // Default to position hold
            control = positionController->computeControl(state, positionTarget);
            break;
    }
    
    // Compute thrust (assuming z-axis is up)
    double mass = 1.0;     // Assumed mass in kg
    double gravity = 9.81; // m/s²
    
    // Simple approximation for thrust
    control.targetThrust = std::min(0.9f, std::max(0.1f, 
        static_cast<float>((control.targetAcceleration.z() + gravity) * mass / 10.0)));
    
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
                currentWaypointIndex = 0;
                waypointReached = false;
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
    
    UAV::logger().Write("TPOS", "TimeUS,X,Y,Z", "Qfff",
                       UAV::logger().getMicroseconds(),
                       static_cast<float>(position.x()),
                       static_cast<float>(position.y()),
                       static_cast<float>(position.z()));
}

// Set velocity target
void UAVStateMachine::setVelocityTarget(const Eigen::Vector3d& velocity) {
    velocityTarget = velocity;
    
    UAV::logger().Write("TVEL", "TimeUS,VX,VY,VZ", "Qfff",
                       UAV::logger().getMicroseconds(),
                       static_cast<float>(velocity.x()),
                       static_cast<float>(velocity.y()),
                       static_cast<float>(velocity.z()));
}

// Set waypoints
void UAVStateMachine::setWaypoints(const std::vector<Waypoint>& newWaypoints) {
    waypoints = newWaypoints;
    currentWaypointIndex = 0;
    waypointReached = false;
    
    UAV::logger().Write("WPLN", "TimeUS,NumWaypoints", "QI",
                       UAV::logger().getMicroseconds(),
                       static_cast<int>(waypoints.size()));
    
    // Log each waypoint
    for (size_t i = 0; i < waypoints.size(); i++) {
        UAV::logger().Write("WYPT", "TimeUS,Index,X,Y,Z", "QIfff",
                           UAV::logger().getMicroseconds(),
                           static_cast<int>(i),
                           static_cast<float>(waypoints[i].position.x()),
                           static_cast<float>(waypoints[i].position.y()),
                           static_cast<float>(waypoints[i].position.z()));
    }
}

// Check if position has been reached
bool UAVStateMachine::isPositionReached(const Eigen::Vector3d& current, 
                                      const Eigen::Vector3d& target,
                                      double threshold) {
    return (current - target).norm() < threshold;
}