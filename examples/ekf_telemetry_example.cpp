#include "Telemetry.h"
#include "vision/pipelines/aruco_ekf_estimator.h"
#include <iostream>

// Simple EKF telemetry integration example
class EKFTelemetryIntegration {
public:
    static void logEKFState(const EKFStateResult& ekfState) {
        if (!ekfState.valid) {
            TEL_BOOL("ekf/valid", false);
            return;
        }
        
        // Log validity
        TEL_BOOL("ekf/valid", true);
        
        // Log position
        TEL_VECTOR("ekf/position", ekfState.position.x(), ekfState.position.y(), ekfState.position.z());
        TEL_NUMBER("ekf/position_x", ekfState.position.x());
        TEL_NUMBER("ekf/position_y", ekfState.position.y());
        TEL_NUMBER("ekf/position_z", ekfState.position.z());
        
        // Log velocity
        TEL_VECTOR("ekf/velocity", ekfState.velocity.x(), ekfState.velocity.y(), ekfState.velocity.z());
        TEL_NUMBER("ekf/velocity_x", ekfState.velocity.x());
        TEL_NUMBER("ekf/velocity_y", ekfState.velocity.y());
        TEL_NUMBER("ekf/velocity_z", ekfState.velocity.z());
        
        // Log acceleration
        TEL_VECTOR("ekf/acceleration", ekfState.acceleration.x(), ekfState.acceleration.y(), ekfState.acceleration.z());
        TEL_NUMBER("ekf/acceleration_x", ekfState.acceleration.x());
        TEL_NUMBER("ekf/acceleration_y", ekfState.acceleration.y());
        TEL_NUMBER("ekf/acceleration_z", ekfState.acceleration.z());
        
        // Log uncertainties (standard deviations)
        TEL_VECTOR("ekf/position_std", ekfState.positionStdDev.x(), ekfState.positionStdDev.y(), ekfState.positionStdDev.z());
        TEL_VECTOR("ekf/velocity_std", ekfState.velocityStdDev.x(), ekfState.velocityStdDev.y(), ekfState.velocityStdDev.z());
        TEL_VECTOR("ekf/acceleration_std", ekfState.accelerationStdDev.x(), ekfState.accelerationStdDev.y(), ekfState.accelerationStdDev.z());
        
        // Log derived metrics
        double position_magnitude = ekfState.position.norm();
        double velocity_magnitude = ekfState.velocity.norm();
        double acceleration_magnitude = ekfState.acceleration.norm();
        
        TEL_NUMBER("ekf/position_magnitude", position_magnitude);
        TEL_NUMBER("ekf/velocity_magnitude", velocity_magnitude);
        TEL_NUMBER("ekf/acceleration_magnitude", acceleration_magnitude);
        
        // Log uncertainty metrics
        double position_uncertainty = ekfState.positionStdDev.norm();
        double velocity_uncertainty = ekfState.velocityStdDev.norm();
        
        TEL_NUMBER("ekf/position_uncertainty", position_uncertainty);
        TEL_NUMBER("ekf/velocity_uncertainty", velocity_uncertainty);
        
        // Log confidence metrics (inverse of uncertainty)
        TEL_NUMBER("ekf/position_confidence", 1.0 / (1.0 + position_uncertainty));
        TEL_NUMBER("ekf/velocity_confidence", 1.0 / (1.0 + velocity_uncertainty));
        
        // Timestamp
        auto now = std::chrono::steady_clock::now();
        auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        TEL_NUMBER("ekf/timestamp_ms", static_cast<double>(timestamp_ms));
    }
    
    // Enhanced logging with derived metrics
    static void logEKFStateEnhanced(const EKFStateResult& ekfState, const std::string& prefix = "ekf") {
        if (!ekfState.valid) {
            TEL_BOOL(prefix + "/valid", false);
            TEL_EVENT("EKF state invalid");
            return;
        }
        
        TEL_FUNCTION_TIMER();
        
        // Basic state
        logEKFState(ekfState);
        
        // Additional derived metrics using helper functions
        TelH::logPosition(prefix, ekfState.position.x(), ekfState.position.y(), ekfState.position.z());
        TelH::logVelocity(prefix, ekfState.velocity.x(), ekfState.velocity.y(), ekfState.velocity.z());
        
        // Quality metrics
        double max_position_std = std::max({ekfState.positionStdDev.x(), ekfState.positionStdDev.y(), ekfState.positionStdDev.z()});
        double max_velocity_std = std::max({ekfState.velocityStdDev.x(), ekfState.velocityStdDev.y(), ekfState.velocityStdDev.z()});
        
        TEL_NUMBER(prefix + "/quality/max_position_std", max_position_std);
        TEL_NUMBER(prefix + "/quality/max_velocity_std", max_velocity_std);
        
        // Health assessment
        bool position_healthy = max_position_std < 0.5;  // Adjust threshold as needed
        bool velocity_healthy = max_velocity_std < 1.0;  // Adjust threshold as needed
        bool overall_healthy = position_healthy && velocity_healthy;
        
        TelH::logHealth(prefix, overall_healthy, 
                       overall_healthy ? "EKF state good" : "EKF state uncertain");
        
        if (!overall_healthy) {
            if (!position_healthy) {
                TEL_WARNING("High EKF position uncertainty: " + std::to_string(max_position_std));
            }
            if (!velocity_healthy) {
                TEL_WARNING("High EKF velocity uncertainty: " + std::to_string(max_velocity_std));
            }
        }
    }
};

int main() {
    std::cout << "EKF Telemetry Integration Test" << std::endl;
    
    // Initialize telemetry
    if (!TelemetryInit::initialize(14559, 30)) {
        std::cerr << "Failed to initialize telemetry!" << std::endl;
        return 1;
    }
    
    TEL_INFO("EKF Telemetry test started");
    
    // Simulate EKF data
    for (int i = 0; i < 200; ++i) {
        // Create sample EKF state
        EKFStateResult ekfState;
        ekfState.valid = true;
        
        // Simulate some trajectory
        double t = i * 0.05; // 50ms timesteps
        ekfState.position = Eigen::Vector3d(std::sin(t * 0.1), std::cos(t * 0.1), 0.5 + 0.1 * std::sin(t * 0.3));
        ekfState.velocity = Eigen::Vector3d(0.1 * std::cos(t * 0.1), -0.1 * std::sin(t * 0.1), 0.03 * std::cos(t * 0.3));
        ekfState.acceleration = Eigen::Vector3d(-0.01 * std::sin(t * 0.1), -0.01 * std::cos(t * 0.1), -0.009 * std::sin(t * 0.3));
        
        // Add some realistic uncertainty
        double base_pos_std = 0.1 + 0.05 * std::sin(t * 0.2);
        double base_vel_std = 0.2 + 0.1 * std::sin(t * 0.15);
        
        ekfState.positionStdDev = Eigen::Vector3d::Constant(base_pos_std);
        ekfState.velocityStdDev = Eigen::Vector3d::Constant(base_vel_std);
        ekfState.accelerationStdDev = Eigen::Vector3d::Constant(0.5);
        
        ekfState.timestamp = std::chrono::steady_clock::now();
        
        // Log the EKF state using our telemetry integration
        EKFTelemetryIntegration::logEKFStateEnhanced(ekfState);
        
        // Occasionally simulate invalid state
        if (i % 50 == 49) {
            EKFStateResult invalidState;
            invalidState.valid = false;
            EKFTelemetryIntegration::logEKFState(invalidState);
            TEL_ERROR("EKF state became invalid at iteration " + std::to_string(i));
        }
        
        // Log iteration info
        if (i % 20 == 0) {
            TEL_INFO("Processed " + std::to_string(i) + " EKF updates");
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    TEL_INFO("EKF Telemetry test completed");
    
    // Shutdown
    TelemetryInit::shutdown();
    
    std::cout << "Test completed. Check the telemetry visualizer for results." << std::endl;
    
    return 0;
}
