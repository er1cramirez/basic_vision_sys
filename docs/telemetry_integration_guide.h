#ifndef TELEMETRY_INTEGRATION_GUIDE_H
#define TELEMETRY_INTEGRATION_GUIDE_H

/**
 * @file telemetry_integration_guide.h
 * @brief Examples showing how to integrate the generic telemetry system into existing code
 * 
 * This file shows practical examples of adding telemetry to your existing UAV control,
 * vision, and other systems with minimal code changes.
 */

#include "Telemetry.h"

//==============================================================================
// EXAMPLE 1: Adding telemetry to existing UAV controller
//==============================================================================

// BEFORE: Original controller without telemetry
class OriginalUAVController {
public:
    void update(const Vector3d& reference, const Vector3d& current_position) {
        Vector3d error = reference - current_position;
        Vector3d control_output = pid_controller.update(error);
        
        // Send control output to actuators
        sendControlCommand(control_output);
    }
};

// AFTER: Same controller with telemetry added (only 4 lines added!)
class TelemetryEnabledUAVController {
public:
    void update(const Vector3d& reference, const Vector3d& current_position) {
        TEL_FUNCTION_TIMER();  // +1 line: automatic timing
        
        Vector3d error = reference - current_position;
        Vector3d control_output = pid_controller.update(error);
        
        // +3 lines: log key data for visualization
        TEL_VECTOR("control/reference", reference.x(), reference.y(), reference.z());
        TEL_VECTOR("control/position", current_position.x(), current_position.y(), current_position.z());
        TEL_VECTOR("control/output", control_output.x(), control_output.y(), control_output.z());
        
        // Send control output to actuators
        sendControlCommand(control_output);
    }
};

//==============================================================================
// EXAMPLE 2: Adding telemetry to vision pipeline
//==============================================================================

// BEFORE: Vision pipeline without telemetry
class OriginalVisionPipeline {
public:
    bool processFrame(const cv::Mat& frame) {
        auto start = std::chrono::steady_clock::now();
        
        bool target_detected = detectArUcoMarker(frame);
        
        auto end = std::chrono::steady_clock::now();
        double process_time = std::chrono::duration<double, std::milli>(end - start).count();
        
        return target_detected;
    }
};

// AFTER: Same pipeline with telemetry (minimal changes)
class TelemetryEnabledVisionPipeline {
public:
    bool processFrame(const cv::Mat& frame) {
        TEL_AUTO_TIMER("vision_processing");  // Replaces manual timing
        
        bool target_detected = detectArUcoMarker(frame);
        
        // +2 lines: log results
        TEL_BOOL("vision/target_detected", target_detected);
        TEL_FPS("vision", calculateFPS());
        
        return target_detected;
    }
};

//==============================================================================
// EXAMPLE 3: Adding telemetry to state machine
//==============================================================================

// BEFORE: State machine without telemetry  
class OriginalStateMachine {
    enum State { IDLE, TAKEOFF, FLYING, LANDING };
    State current_state = IDLE;
    
public:
    void transition_to(State new_state) {
        current_state = new_state;
        std::cout << "State changed to " << state_name(new_state) << std::endl;
    }
};

// AFTER: State machine with telemetry
class TelemetryEnabledStateMachine {
    enum State { IDLE, TAKEOFF, FLYING, LANDING };
    State current_state = IDLE;
    
public:
    void transition_to(State new_state) {
        current_state = new_state;
        
        // +2 lines: log state changes
        TEL_STRING("state/current", state_name(new_state));
        TEL_EVENT("State transition: " + state_name(current_state) + " -> " + state_name(new_state));
    }
};

//==============================================================================
// EXAMPLE 4: Adding telemetry to existing main loop
//==============================================================================

// BEFORE: Main control loop
void original_main_loop() {
    while (running) {
        // Get sensor data
        Vector3d position = get_position();
        Vector3d velocity = get_velocity();
        
        // Run control
        controller.update(reference_position, position);
        
        // Process vision
        bool target_visible = vision.processFrame(camera.getFrame());
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// AFTER: Same loop with comprehensive telemetry
void telemetry_enabled_main_loop() {
    // +1 line: initialize telemetry
    TelemetryInit::initialize(14559, 50);  // 50 Hz telemetry
    
    while (running) {
        TEL_AUTO_TIMER("main_loop");  // +1 line: time the loop
        
        // Get sensor data
        Vector3d position = get_position();
        Vector3d velocity = get_velocity();
        
        // +2 lines: log sensor data
        TEL_VECTOR("sensors/position", position.x(), position.y(), position.z());
        TEL_VECTOR("sensors/velocity", velocity.x(), velocity.y(), velocity.z());
        
        // Run control
        controller.update(reference_position, position);
        
        // Process vision
        bool target_visible = vision.processFrame(camera.getFrame());
        
        // +1 line: log system health
        TelH::logHealth("system", target_visible && position.norm() < 10.0);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    TelemetryInit::shutdown();  // +1 line: cleanup
}

//==============================================================================
// EXAMPLE 5: Retrofitting existing error handling
//==============================================================================

// BEFORE: Basic error handling
class OriginalErrorHandler {
public:
    void handle_error(const std::string& error_msg) {
        std::cerr << "ERROR: " << error_msg << std::endl;
        error_count++;
    }
    
    void handle_warning(const std::string& warning_msg) {
        std::cout << "WARNING: " << warning_msg << std::endl;
        warning_count++;
    }
};

// AFTER: Error handling with telemetry logging
class TelemetryEnabledErrorHandler {
public:
    void handle_error(const std::string& error_msg) {
        std::cerr << "ERROR: " << error_msg << std::endl;
        error_count++;
        
        // +2 lines: log errors for visualization
        TEL_ERROR(error_msg);
        TEL_NUMBER("diagnostics/error_count", error_count);
    }
    
    void handle_warning(const std::string& warning_msg) {
        std::cout << "WARNING: " << warning_msg << std::endl;
        warning_count++;
        
        // +2 lines: log warnings
        TEL_WARNING(warning_msg);
        TEL_NUMBER("diagnostics/warning_count", warning_count);
    }
};

//==============================================================================
// EXAMPLE 6: Quick retrofit for any existing function
//==============================================================================

// BEFORE: Any existing function
double calculate_trajectory_error(const Vector3d& reference, const Vector3d& actual) {
    double error = (reference - actual).norm();
    return error;
}

// AFTER: Same function with telemetry (2 lines added)
double calculate_trajectory_error_with_telemetry(const Vector3d& reference, const Vector3d& actual) {
    TEL_FUNCTION_TIMER();  // +1 line: automatic timing
    
    double error = (reference - actual).norm();
    
    TEL_NUMBER("navigation/trajectory_error", error);  // +1 line: log result
    
    return error;
}

//==============================================================================
// INTEGRATION CHECKLIST
//==============================================================================

/**
 * Quick integration checklist for adding telemetry to existing code:
 * 
 * 1. □ Add #include "Telemetry.h" to relevant files
 * 2. □ Add TelemetryInit::initialize() to main() or setup function
 * 3. □ Add TEL_FUNCTION_TIMER() to functions you want to profile
 * 4. □ Add TEL_NUMBER(), TEL_VECTOR(), etc. for key variables
 * 5. □ Replace std::cout debug messages with TEL_INFO(), TEL_WARNING()
 * 6. □ Add TEL_EVENT() for important state changes
 * 7. □ Add TelemetryInit::shutdown() before program exit
 * 8. □ Run python3 tools/generic_telemetry_viewer.py to view data
 * 
 * Typical effort: 5-10 lines of code changes for comprehensive telemetry!
 */

#endif // TELEMETRY_INTEGRATION_GUIDE_H
