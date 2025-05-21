#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <chrono>

/**
 * @struct ControlOutput
 * @brief Output from the control system to be sent to the drone
 */
struct ControlOutput {
    // Drone force control
    Eigen::Vector3d u_desired; // Desired control output
    Eigen::Vector3d u_desired_dot; // Desired control output derivative
    
    // Timestamp when the control was computed
    std::chrono::steady_clock::time_point timestamp;
    
    // Constructor with defaults
    ControlOutput() 
        : u_desired(Eigen::Vector3d::Zero()),
          u_desired_dot(Eigen::Vector3d::Zero()) {
        timestamp = std::chrono::steady_clock::now();
    }
};

#endif // CONTROL_TYPES_H