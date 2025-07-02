/**
 * @file test_mavlink_frequency.cpp
 * @brief Test to verify MavlinkCommModule quaternion frequency
 */

#include "mavlink_comm_module.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>

volatile bool running = true;

void signal_handler(int signum) {
    std::cout << "\nReceived signal " << signum << ", shutting down..." << std::endl;
    running = false;
}

int main() {
    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "=== MavlinkCommModule Quaternion Frequency Test ===" << std::endl;
    std::cout << "Make sure MAVProxy is running with:" << std::endl;
    std::cout << "mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550" << std::endl;
    std::cout << "And quaternion messages are enabled at 50Hz" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "=======================================" << std::endl;
    
    // Create MavlinkCommModule
    MavlinkCommModule mavlink("127.0.0.1", 14550);
    
    // Start the module
    if (!mavlink.start()) {
        std::cerr << "Failed to start MAVLink module" << std::endl;
        return 1;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    auto last_status_time = start_time;
    
    std::cout << "\nStarting frequency test..." << std::endl;
    
    while (running) {
        // Get latest attitude data
        AttitudeData attitude_data;
        bool have_data = mavlink.getLatestAttitude(attitude_data);
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(current_time - last_status_time).count();
        
        // Print status every 3 seconds
        if (elapsed >= 3.0) {
            if (have_data) {
                std::cout << "[TEST] Attitude: valid=" << attitude_data.valid 
                          << ", freq=" << attitude_data.frequency_hz << "Hz"
                          << ", msgs=" << attitude_data.message_count 
                          << ", age=" << attitude_data.getAgeMs() << "ms" << std::endl;
                std::cout << "[TEST] Quaternion: q1=" << attitude_data.q1 
                          << ", q2=" << attitude_data.q2 
                          << ", q3=" << attitude_data.q3 
                          << ", q4=" << attitude_data.q4 << std::endl;
                std::cout << "[TEST] Euler: roll=" << attitude_data.roll_deg 
                          << "°, pitch=" << attitude_data.pitch_deg 
                          << "°, yaw=" << attitude_data.yaw_deg << "°" << std::endl;
            } else {
                std::cout << "[TEST] No attitude data available" << std::endl;
            }
            last_status_time = current_time;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "\nStopping MAVLink module..." << std::endl;
    mavlink.stop();
    
    // Final report
    AttitudeData final_data;
    if (mavlink.getLatestAttitude(final_data)) {
        auto total_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        std::cout << "\n=== FINAL RESULTS ===" << std::endl;
        std::cout << "Test duration: " << total_time << "s" << std::endl;
        std::cout << "Total messages: " << final_data.message_count << std::endl;
        std::cout << "Final frequency: " << final_data.frequency_hz << "Hz" << std::endl;
        
        if (final_data.frequency_hz >= 45.0) {
            std::cout << "✅ QUATERNION FREQUENCY TEST PASSED" << std::endl;
        } else {
            std::cout << "❌ QUATERNION FREQUENCY TEST FAILED" << std::endl;
        }
    }
    
    return 0;
}
