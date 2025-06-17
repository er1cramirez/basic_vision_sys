#include "SimpleTelemetry.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>

int main() {
    std::cout << "Testing SimpleTelemetry system..." << std::endl;
    
    // Initialize telemetry
    if (!SimpleTelemetry::initialize(14559)) {
        std::cerr << "Failed to initialize telemetry!" << std::endl;
        return 1;
    }
    
    std::cout << "Telemetry initialized. Sending test data..." << std::endl;
    std::cout << "Start the Python viewer: python3 tools/generic_telemetry_viewer.py" << std::endl;
    
    // Send test data for 30 seconds
    auto start_time = std::chrono::steady_clock::now();
    double t = 0.0;
    
    while (t < 30.0) {
        // Simulate EKF-like data
        double pos_x = 5.0 * sin(t * 0.1);
        double pos_y = 3.0 * cos(t * 0.15);
        double pos_z = 1.0 + 0.5 * sin(t * 0.2);
        
        double vel_x = 0.5 * cos(t * 0.1);
        double vel_y = -0.45 * sin(t * 0.15);
        double vel_z = 0.1 * cos(t * 0.2);
        
        double acc_x = -0.05 * sin(t * 0.1);
        double acc_y = -0.0675 * cos(t * 0.15);
        double acc_z = -0.02 * sin(t * 0.2);
        
        // Send position data
        TEL_VECTOR("ekf/position", pos_x, pos_y, pos_z);
        TEL_NUMBER("ekf/position_x", pos_x);
        TEL_NUMBER("ekf/position_y", pos_y);
        TEL_NUMBER("ekf/position_z", pos_z);
        TEL_NUMBER("ekf/position_magnitude", sqrt(pos_x*pos_x + pos_y*pos_y + pos_z*pos_z));
        
        // Send velocity data
        TEL_VECTOR("ekf/velocity", vel_x, vel_y, vel_z);
        TEL_NUMBER("ekf/velocity_x", vel_x);
        TEL_NUMBER("ekf/velocity_y", vel_y);
        TEL_NUMBER("ekf/velocity_z", vel_z);
        TEL_NUMBER("ekf/velocity_magnitude", sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z));
        
        // Send acceleration data
        TEL_VECTOR("ekf/acceleration", acc_x, acc_y, acc_z);
        TEL_NUMBER("ekf/acceleration_x", acc_x);
        TEL_NUMBER("ekf/acceleration_y", acc_y);
        TEL_NUMBER("ekf/acceleration_z", acc_z);
        TEL_NUMBER("ekf/acceleration_magnitude", sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z));
        
        // Send status data
        TEL_BOOL("ekf/valid", true);
        TEL_NUMBER("system/time", t);
        TEL_STRING("system/status", "Running test");
        
        // Update time
        auto current_time = std::chrono::steady_clock::now();
        t = std::chrono::duration<double>(current_time - start_time).count();
        
        std::cout << "\rTime: " << std::fixed << std::setprecision(1) << t << "s" << std::flush;
        
        // Sleep for 50ms (20 Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "\nTest completed. Shutting down..." << std::endl;
    SimpleTelemetry::shutdown();
    return 0;
}
