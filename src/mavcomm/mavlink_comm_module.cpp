/**
 * @file mavlink_comm_module.cpp
 * @brief Implementation of MAVLink communication module
 */

#include "mavlink_comm_module.h"
#include "Logger.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <map>
#include <thread>
#include <chrono>

// UDP Constructor
MavlinkCommModule::MavlinkCommModule(const std::string& target_ip, int udp_port, 
                                    uint8_t system_id, uint8_t component_id,
                                    uint8_t target_system_id, uint8_t target_component_id)
    : using_udp_(true),
      system_id_(system_id),
      component_id_(component_id),
      target_system_id_(target_system_id),
      target_component_id_(target_component_id) {
    
    // Create UDP port
    port_ = std::make_unique<UDP_Port>(target_ip.c_str(), udp_port);
    
    std::cout << "Initialized UDP MAVLink communication on " << target_ip << ":" << udp_port << std::endl;
    std::cout << "System ID: " << static_cast<int>(system_id_) 
              << ", Component ID: " << static_cast<int>(component_id_) << std::endl;
}

// Serial Constructor
MavlinkCommModule::MavlinkCommModule(const std::string& uart_name, int baudrate,
                                    uint8_t system_id, uint8_t component_id,
                                    uint8_t target_system_id, uint8_t target_component_id,
                                    bool /* is_serial_flag */)
    : using_udp_(false),
      system_id_(system_id),
      component_id_(component_id),
      target_system_id_(target_system_id),
      target_component_id_(target_component_id) {
    
    // Create Serial port
    port_ = std::make_unique<Serial_Port>(uart_name.c_str(), baudrate);
    
    std::cout << "Initialized Serial MAVLink communication on " << uart_name 
              << " at " << baudrate << " baud" << std::endl;
    std::cout << "System ID: " << static_cast<int>(system_id_) 
              << ", Component ID: " << static_cast<int>(component_id_) << std::endl;
}

// Destructor
MavlinkCommModule::~MavlinkCommModule() {
    stop();
}

// Start communication
bool MavlinkCommModule::start() {
    if (running_) {
        std::cout << "Communication already running" << std::endl;
        return true;
    }
    
    try {
        // Start port
        port_->start();
        
        // Set running flag
        running_ = true;
        
        // Start threads
        send_thread_ = std::thread(&MavlinkCommModule::sendThreadFunc, this);
        receive_thread_ = std::thread(&MavlinkCommModule::receiveThreadFunc, this);
        
        std::cout << "MAVLink communication started successfully" << std::endl;
        
        // Note: Quaternion message frequency should be set externally via Python script
        // Use: python3 tools/msg_setup.py <frequency_hz> to configure ATTITUDE_QUATERNION rate
        // This avoids conflicts and allows proper frequency control
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error starting MAVLink communication: " << e.what() << std::endl;
        stop();
        return false;
    }
}

// Stop communication
void MavlinkCommModule::stop() {
    if (!running_) {
        return;
    }
    
    // Set running flag to false to stop threads
    running_ = false;
    
    // Join threads if they're joinable
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    // Stop port
    if (port_ && port_->is_running()) {
        port_->stop();
    }
    
    std::cout << "MAVLink communication stopped" << std::endl;
}

// Check if running
bool MavlinkCommModule::isRunning() const {
    return running_ && port_->is_running();
}

// Set frequency
void MavlinkCommModule::setFrequency(double frequency_hz) {
    if (frequency_hz <= 0) {
        std::cerr << "Warning: Invalid frequency, must be > 0. Using 1Hz instead." << std::endl;
        frequency_hz = 1.0;
    }
    
    std::lock_guard<std::mutex> lock(frequency_mutex_);
    frequency_hz_ = frequency_hz;
    updatePeriod();
    
    std::cout << "Set sending frequency to " << frequency_hz_ << " Hz (period: " 
              << period_.count() << " µs)" << std::endl;
}

// Get frequency
double MavlinkCommModule::getFrequency() const {
    std::lock_guard<std::mutex> lock(frequency_mutex_);
    return frequency_hz_;
}

// Update period based on frequency
void MavlinkCommModule::updatePeriod() {
    period_ = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / frequency_hz_));
}

// Set force vector
void MavlinkCommModule::setForceVector(float force_x, float force_y, float force_z,
                                      float force_derivative_x, float force_derivative_y, 
                                      float force_derivative_z) {
    std::lock_guard<std::mutex> lock(force_vector_mutex_);
    force_x_ = force_x;
    force_y_ = force_y;
    force_z_ = force_z;
    force_derivative_x_ = force_derivative_x;
    force_derivative_y_ = force_derivative_y;
    force_derivative_z_ = force_derivative_z;
}

// Compose the force vector message
mavlink_message_t MavlinkCommModule::composeForceVectorMessage() {
    mavlink_message_t message;
    
    // Lock to safely access the force vector data
    std::lock_guard<std::mutex> lock(force_vector_mutex_);
    
    // Compose the message
    mavlink_msg_force_vector_target_pack(
        system_id_,              // System ID
        component_id_,           // Component ID
        &message,                // Message to fill
        target_system_id_,       // Target system
        target_component_id_,    // Target component
        msg_seq_++,              // Message sequence
        force_x_,                // Force X
        force_y_,                // Force Y
        force_z_,                // Force Z
        force_derivative_x_,     // Force derivative X
        force_derivative_y_,     // Force derivative Y
        force_derivative_z_      // Force derivative Z
    );
    
    return message;
}

// Send force vector immediately
bool MavlinkCommModule::sendForceVectorNow() {
    if (!isRunning()) {
        std::cerr << "Error: Cannot send message, communication not running" << std::endl;
        return false;
    }
    
    mavlink_message_t message = composeForceVectorMessage();
    int result = port_->write_message(message);
    
    return (result > 0);
}

// Register callback for received messages
void MavlinkCommModule::registerMessageCallback(std::function<void(const mavlink_message_t&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    message_callback_ = callback;
}

// Sending thread function
void MavlinkCommModule::sendThreadFunc() {
    std::cout << "Send thread started, frequency: " << getFrequency() << " Hz" << std::endl;
    
    auto next_send_time = std::chrono::steady_clock::now();
    
    while (running_) {
        // Wait until next scheduled time
        std::this_thread::sleep_until(next_send_time);
        
        // Send the message if we're still running
        if (running_) {
            if (!sendForceVectorNow()) {
                std::cerr << "Warning: Failed to send force vector message" << std::endl;
            }
        }
        
        // Update next send time
        {
            std::lock_guard<std::mutex> lock(frequency_mutex_);
            next_send_time += period_;
        }
        
        // Check if we're falling behind
        auto now = std::chrono::steady_clock::now();
        if (now > next_send_time) {
            // We're falling behind, reset timing
            std::cerr << "Warning: Send thread falling behind schedule, resetting timing" << std::endl;
            next_send_time = now + period_;
        }
    }
    
    std::cout << "Send thread stopped" << std::endl;
}

// Receiving thread function
void MavlinkCommModule::receiveThreadFunc() {
    std::cout << "Receive thread started" << std::endl;
    
    mavlink_message_t message;
    
    while (running_) {
        // Try to read a message
        if (port_->read_message(message)) {
            // Debug: Print ALL received message IDs to understand what we're getting
            static std::map<uint32_t, int> msg_id_counts;
            msg_id_counts[message.msgid]++;
            
            static int total_msg_count = 0;
            total_msg_count++;
            switch (message.msgid) {
                default:
                    // Log unhandled message types occasionally for debugging
                    if (msg_id_counts[message.msgid] == 1) {
                        std::cout << "First time seeing message ID: " << message.msgid << std::endl;
                    }
                    break;
            }
            
            // Process message through callback if registered
            std::function<void(const mavlink_message_t&)> callback;
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                callback = message_callback_;
            }
            
            // Call the callback if registered
            if (callback) {
                callback(message);
            }
        }
        
        // Small sleep to avoid consuming 100% CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cout << "Receive thread stopped" << std::endl;
}

// NOTE: All quaternion/attitude methods removed - now handled exclusively by SimpleQuaternionReader

// NOTE: All quaternion/attitude methods removed - now handled exclusively by SimpleQuaternionReader

// NOTE: processAttitudeMessage removed - quaternion processing now handled by SimpleQuaternionReader