#include "Telemetry.h"  // Just include this one header!
#include <iostream>
#include <thread>
#include <chrono>
#include <random>

// Example showing the super simple telemetry API
class SimpleControlExample {
private:
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist;
    
    struct State {
        double position[3] = {0, 0, 0};
        double velocity[3] = {0, 0, 0};
        double target[3] = {1, 1, 1};
    } state;
    
public:
    SimpleControlExample() : rng(std::random_device{}()), dist(-1.0, 1.0) {}
    
    void runExample() {
        std::cout << "Running simple telemetry example..." << std::endl;
        std::cout << "View at http://localhost:8050" << std::endl;
        
        for (int i = 0; i < 500; ++i) {
            // Automatic timing of the entire control loop
            TEL_FUNCTION_TIMER();
            
            // Update system
            updateSystem();
            
            // Log all data with one-liners
            TEL_NUMBER("iteration", i);
            TEL_VECTOR("state/position", state.position[0], state.position[1], state.position[2]);
            TEL_VECTOR("state/velocity", state.velocity[0], state.velocity[1], state.velocity[2]);
            TEL_VECTOR("control/target", state.target[0], state.target[1], state.target[2]);
            
            // Use helper functions for common patterns
            TelH::logError("control", 
                          state.target[0], state.target[1], state.target[2],
                          state.position[0], state.position[1], state.position[2]);
            
            // Log health status
            double error_mag = sqrt(pow(state.target[0] - state.position[0], 2) +
                                   pow(state.target[1] - state.position[1], 2) +
                                   pow(state.target[2] - state.position[2], 2));
            bool healthy = error_mag < 0.5;
            TelH::logHealth("control_system", healthy, 
                           healthy ? "Tracking well" : "High error");
            
            // Log events occasionally
            if (i % 100 == 0) {
                TEL_EVENT("Completed " + std::to_string(i) + " iterations");
            }
            
            // Change target occasionally
            if (i % 150 == 0) {
                state.target[0] = dist(rng) * 2.0;
                state.target[1] = dist(rng) * 2.0;
                state.target[2] = dist(rng) * 2.0;
                TEL_INFO("Target changed to: [" + 
                        std::to_string(state.target[0]) + ", " +
                        std::to_string(state.target[1]) + ", " +
                        std::to_string(state.target[2]) + "]");
            }
            
            // Simulate work with timing
            {
                TEL_AUTO_TIMER("vision_processing");
                std::this_thread::sleep_for(std::chrono::milliseconds(5 + rand() % 10));
                
                // Log vision results
                TEL_BOOL("vision/target_detected", true);
                TEL_NUMBER("vision/confidence", 0.8 + (rand() % 20) / 100.0);
            }
            
            // Log performance
            TEL_FPS("control_system", 50.0);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        TEL_INFO("Example completed successfully");
    }
    
private:
    void updateSystem() {
        // Simple control simulation
        for (int i = 0; i < 3; ++i) {
            double error = state.target[i] - state.position[i];
            state.velocity[i] = error * 2.0 + dist(rng) * 0.1; // P controller + noise
            state.position[i] += state.velocity[i] * 0.02; // integrate
        }
    }
};

int main() {
    // Initialize telemetry (this happens automatically, but you can be explicit)
    if (!TelemetryInit::initialize(14559, 30)) {
        std::cerr << "Failed to initialize telemetry!" << std::endl;
        return 1;
    }
    
    TEL_INFO("Starting simple telemetry example");
    
    // Run the example
    SimpleControlExample example;
    example.runExample();
    
    // Shutdown telemetry
    TelemetryInit::shutdown();
    
    return 0;
}
