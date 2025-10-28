/**
 * @file velocity_pi_controller_example.cpp
 * @brief Example demonstrating the use of VelocityPIController
 * 
 * This example shows how to use the VelocityPIController with integral term
 * and delay mechanism to avoid excessive accumulation during startup.
 */

#include "control/Controllers.h"
#include "aruco_ekf_estimator.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "VelocityPIController Example\n";
    std::cout << "============================\n\n";

    // Create the PI controller with custom gains
    Eigen::Vector3d kp(-0.015, -0.01, -0.008);  // Different gains per axis [x, y, z]
    Eigen::Vector3d ki(-0.008, -0.005, -0.003); // Different integral gains per axis [x, y, z]
    
    VelocityPIController controller(kp, ki);
    
    // Initialize the controller
    if (!controller.initialize()) {
        std::cerr << "Failed to initialize controller\n";
        return -1;
    }
    
    std::cout << "Controller initialized successfully\n";
    std::cout << "Proportional gains: [" << kp.x() << ", " << kp.y() << ", " << kp.z() << "]\n";
    std::cout << "Integral gains: [" << ki.x() << ", " << ki.y() << ", " << ki.z() << "]\n";
    std::cout << "Integral action: Active on all axes from start\n\n";
    
    // Example: Tune individual axis gains during runtime
    std::cout << "Tuning Z-axis gains for better altitude control...\n";
    controller.setAxisGains(2, -0.012, -0.004); // Z-axis (index 2) with different gains
    std::cout << "All axes have integral action active from start\n\n";
    
    // Simulate some control loop iterations
    Eigen::Vector3d target(1.0, 1.0, 0.0); // Target position
    
    // Create a mock state estimate
    EKFStateResult state;
    state.position = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
    
    std::cout << "Starting control simulation...\n";
    std::cout << "Target position: [" << target.x() << ", " << target.y() << ", " << target.z() << "]\n\n";
    
    // Simulate control loop for 5 seconds
    auto start_time = std::chrono::steady_clock::now();
    int iteration = 0;
    
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < 5.0) {
        // Compute control output
        ControlOutput output = controller.computeControl(state, target);
        
        // Print results every 10 iterations (approximately every 0.5 seconds)
        if (iteration % 10 == 0) {
            auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            std::cout << "Time: " << std::fixed << std::setprecision(2) << elapsed << "s, ";
            std::cout << "Control output: [" << std::fixed << std::setprecision(4) 
                      << output.u_desired.x() << ", " 
                      << output.u_desired.y() << ", " 
                      << output.u_desired.z() << "]\n";
        }
        
        // Simple integration to simulate drone movement (for demonstration)
        double dt = 0.05; // 50ms control loop
        state.velocity += output.u_desired * dt;
        state.position += state.velocity * dt;
        
        iteration++;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "\nSimulation completed!\n";
    std::cout << "Final position: [" << std::fixed << std::setprecision(4) 
              << state.position.x() << ", " 
              << state.position.y() << ", " 
              << state.position.z() << "]\n";
    
    return 0;
}
