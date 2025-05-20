#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <chrono>


/**
 * @struct DroneControlOutput
 * @brief Output from the control system to be sent to the drone
 */
struct DroneControlOutput {
    // u_desired
    Eigen::Vector3d u_desired;  // Desired control output
    // Derivative of u_desired
    Eigen::Vector3d u_desired_dot;  // Time derivative of desired control output

    DroneControlOutput() 
        : u_desired(Eigen::Vector3d::Zero()), 
          u_desired_dot(Eigen::Vector3d::Zero()) {}
};
/**
 * @struct ControllerPerformanceData
 * @brief Performance data for the controller
 */
struct ControllerPerformanceData {
    double time;  // Time of the data point
    double error;  // Error in the control output
    double controlEffort;  // Control effort used

    ControllerPerformanceData() 
        : time(0.0), error(0.0), controlEffort(0.0) {}
};
#endif // CONTROL_TYPES_H