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
    // Target values
    Eigen::Vector3d targetPosition;      // Target position (m)
    Eigen::Vector3d targetVelocity;      // Target velocity (m/s)
    Eigen::Vector3d targetAcceleration;  // Target acceleration (m/s²)
    
    // Attitude commands
    Eigen::Vector3d targetAttitudeRPY;   // Roll, pitch, yaw (rad)
    float targetThrust;                  // Normalized thrust [0-1]
    
    // Control mode flags
    bool positionControlEnabled;
    bool velocityControlEnabled;
    bool attitudeControlEnabled;
    bool yawControlEnabled;
    
    // Timestamp when the control was computed
    std::chrono::steady_clock::time_point timestamp;
    
    // Constructor with defaults
    ControlOutput() 
        : targetPosition(Eigen::Vector3d::Zero()),
          targetVelocity(Eigen::Vector3d::Zero()),
          targetAcceleration(Eigen::Vector3d::Zero()),
          targetAttitudeRPY(Eigen::Vector3d::Zero()),
          targetThrust(0.0f),
          positionControlEnabled(false),
          velocityControlEnabled(false),
          attitudeControlEnabled(false),
          yawControlEnabled(false) {
        timestamp = std::chrono::steady_clock::now();
    }
};

/**
 * @struct Waypoint
 * @brief Represents a 3D waypoint for navigation
 */
struct Waypoint {
    Eigen::Vector3d position;      // Position (m)
    Eigen::Vector3d velocity;      // Velocity (m/s)
    float yaw;                     // Yaw angle (rad)
    float dwell_time;              // Time to hold at waypoint (s)
    std::string action;            // Optional action at waypoint
    
    Waypoint(const Eigen::Vector3d& pos, float y = 0.0f, 
             float dwell = 0.0f, const std::string& act = "")
        : position(pos), velocity(Eigen::Vector3d::Zero()),
          yaw(y), dwell_time(dwell), action(act) {}
};

/**
 * @struct MissionPlan
 * @brief Represents a complete mission plan
 */
struct MissionPlan {
    std::vector<Waypoint> waypoints;
    bool repeat;
    
    MissionPlan() : repeat(false) {}
};

#endif // CONTROL_TYPES_H