#include "telemetry/TelemetryPublisher.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <random>

// Example class showing how to integrate telemetry
class ControlSystemExample {
private:
    std::shared_ptr<TelemetryPublisher> telemetry;
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist;
    
    // Example control state
    struct ControlState {
        double position[3] = {0, 0, 0};
        double velocity[3] = {0, 0, 0};
        double reference[3] = {1, 1, 1};
        double control_output[3] = {0, 0, 0};
        double error[3] = {0, 0, 0};
    } state;
    
public:
    ControlSystemExample() : rng(std::random_device{}()), dist(-1.0, 1.0) {
        // Get telemetry publisher instance
        telemetry = Telemetry::getPublisher();
        
        // Initialize telemetry system
        if (!telemetry->initialize(14559, 20)) {
            std::cerr << "Failed to initialize telemetry!" << std::endl;
        }
        
        std::cout << "Control system with telemetry initialized" << std::endl;
    }
    
    void runControlLoop() {
        // Simulate control loop
        for (int i = 0; i < 1000; ++i) {
            // Start timing for this iteration
            Telemetry::startTimer("control_loop");
            
            // Simulate some control calculations
            updateControlSystem();
            
            // Log all the telemetry data
            logTelemetryData();
            
            // End timing
            Telemetry::endTimer("control_loop");
            
            // Log frame rate
            Telemetry::recordFPS("control_system", 50.0); // 50 Hz control loop
            
            // Sleep to simulate real control rate
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            
            if (i % 100 == 0) {
                std::cout << "Control iteration " << i << std::endl;
            }
        }
    }
    
private:
    void updateControlSystem() {
        // Simple PID-like control simulation
        for (int i = 0; i < 3; ++i) {
            // Calculate error
            state.error[i] = state.reference[i] - state.position[i];
            
            // Simple proportional control
            state.control_output[i] = 2.0 * state.error[i];
            
            // Update position (integrate control output with some noise)
            state.velocity[i] = state.control_output[i] + dist(rng) * 0.1;
            state.position[i] += state.velocity[i] * 0.02; // dt = 20ms
        }
        
        // Add some reference changes
        static int counter = 0;
        counter++;
        if (counter % 200 == 0) {
            state.reference[0] = dist(rng) * 2.0;
            state.reference[1] = dist(rng) * 2.0; 
            state.reference[2] = dist(rng) * 2.0;
            
            Telemetry::logEvent("Reference changed to: " + 
                              std::to_string(state.reference[0]) + ", " +
                              std::to_string(state.reference[1]) + ", " +
                              std::to_string(state.reference[2]));
        }
    }
    
    void logTelemetryData() {
        // Method 1: Using convenience functions (recommended for simple data)
        Telemetry::putNumber("control/position_x", state.position[0]);
        Telemetry::putNumber("control/position_y", state.position[1]);
        Telemetry::putNumber("control/position_z", state.position[2]);
        
        Telemetry::putVector3("control/position", state.position[0], state.position[1], state.position[2]);
        Telemetry::putVector3("control/velocity", state.velocity[0], state.velocity[1], state.velocity[2]);
        Telemetry::putVector3("control/reference", state.reference[0], state.reference[1], state.reference[2]);
        Telemetry::putVector3("control/output", state.control_output[0], state.control_output[1], state.control_output[2]);
        Telemetry::putVector3("control/error", state.error[0], state.error[1], state.error[2]);
        
        // Method 2: Using direct publisher methods with units
        telemetry->logScalar("performance/position_error_magnitude", 
                           sqrt(state.error[0]*state.error[0] + state.error[1]*state.error[1] + state.error[2]*state.error[2]), 
                           "m");
        
        telemetry->logScalar("performance/control_effort", 
                           sqrt(state.control_output[0]*state.control_output[0] + 
                                state.control_output[1]*state.control_output[1] + 
                                state.control_output[2]*state.control_output[2]), 
                           "N");
        
        // Method 3: Using specialized entry types
        static auto positionEntry = telemetry->getVectorEntry("state/position_vector", "state", "m");
        static auto velocityEntry = telemetry->getVectorEntry("state/velocity_vector", "state", "m/s");
        
        positionEntry->setValue(state.position[0], state.position[1], state.position[2]);
        velocityEntry->setValue(state.velocity[0], state.velocity[1], state.velocity[2]);
        
        // Method 4: Custom entry with specific categories
        static auto errorEntry = telemetry->getScalarEntry("diagnostics/tracking_error", "diagnostics", "m");
        errorEntry->setValue(sqrt(state.error[0]*state.error[0] + state.error[1]*state.error[1] + state.error[2]*state.error[2]));
        
        // Log boolean status
        bool isStable = abs(state.error[0]) < 0.1 && abs(state.error[1]) < 0.1 && abs(state.error[2]) < 0.1;
        Telemetry::putBoolean("status/is_stable", isStable);
        
        // Log system health
        double maxError = std::max({abs(state.error[0]), abs(state.error[1]), abs(state.error[2])});
        if (maxError > 1.0) {
            Telemetry::logWarning("High tracking error detected: " + std::to_string(maxError));
        } else if (maxError < 0.05) {
            static int goodCounter = 0;
            goodCounter++;
            if (goodCounter % 50 == 0) { // Log every second
                Telemetry::logInfo("System tracking well, error: " + std::to_string(maxError));
            }
        }
    }
};

// Example showing vision system integration
class VisionSystemExample {
private:
    std::shared_ptr<TelemetryPublisher> telemetry;
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist;
    
public:
    VisionSystemExample() : rng(std::random_device{}()), dist(0.0, 1.0) {
        telemetry = Telemetry::getPublisher();
    }
    
    void processFrame(int frameId) {
        Telemetry::startTimer("vision_processing");
        
        // Simulate vision processing
        std::this_thread::sleep_for(std::chrono::milliseconds(10 + static_cast<int>(dist(rng) * 20)));
        
        // Log vision metrics
        double confidence = 0.8 + dist(rng) * 0.2;
        Telemetry::putNumber("vision/detection_confidence", confidence);
        
        bool targetDetected = confidence > 0.85;
        Telemetry::putBoolean("vision/target_detected", targetDetected);
        
        if (targetDetected) {
            // Log target position
            double targetPos[3] = {dist(rng) * 10 - 5, dist(rng) * 10 - 5, dist(rng) * 5};
            Telemetry::putVector3("vision/target_position", targetPos[0], targetPos[1], targetPos[2]);
            
            // Log detection quality metrics
            telemetry->logScalar("vision/pixel_area", 100 + dist(rng) * 500, "pixels");
            telemetry->logScalar("vision/detection_quality", confidence * 100, "%");
        }
        
        Telemetry::endTimer("vision_processing");
        Telemetry::recordFPS("vision_system", 30.0); // 30 Hz vision system
        
        if (frameId % 100 == 0) {
            Telemetry::logInfo("Processed " + std::to_string(frameId) + " vision frames");
        }
    }
};

int main() {
    std::cout << "Starting Generic Telemetry Example" << std::endl;
    
    // Create control system
    ControlSystemExample controlSystem;
    
    // Create vision system  
    VisionSystemExample visionSystem;
    
    // Run systems in parallel
    std::thread controlThread([&controlSystem]() {
        controlSystem.runControlLoop();
    });
    
    std::thread visionThread([&visionSystem]() {
        for (int i = 0; i < 1500; ++i) {
            visionSystem.processFrame(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 Hz
        }
    });
    
    // Wait for completion
    controlThread.join();
    visionThread.join();
    
    // Export all data to file
    auto publisher = Telemetry::getPublisher();
    auto allData = publisher->exportAllData(true);
    
    std::cout << "Telemetry example completed. " << publisher->getEntryCount() 
              << " telemetry entries created." << std::endl;
    
    std::cout << "Entry names: ";
    auto names = publisher->getEntryNames();
    for (const auto& name : names) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    
    // Shutdown
    publisher->shutdown();
    
    return 0;
}
