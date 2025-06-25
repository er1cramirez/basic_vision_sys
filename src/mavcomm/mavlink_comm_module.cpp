/**
 * @file mavlink_comm_module.cpp
 * @brief Implementation of MAVLink communication module
 */

#include "mavlink_comm_module.h"
#include "Logger.h"
#include <iostream>
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
        
        // Debug: Check if we're about to request quaternion
        std::cout << "About to request ATTITUDE_QUATERNION messages..." << std::endl;
        
        // Wait a moment for connection to stabilize, then request ATTITUDE_QUATERNION
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "Finished waiting, now calling requestAttitudeQuaternion..." << std::endl;
        
        requestAttitudeQuaternion(100);  // Request at 10ms interval (100Hz)

        
        std::cout << "Finished calling requestAttitudeQuaternion" << std::endl;
        
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
            
            // // Print summary every 500 messages
            // if (total_msg_count % 500 == 0) {
            //     std::cout << "=== MAVLink Message Summary (total: " << total_msg_count << ") ===" << std::endl;
            //     for (const auto& [msg_id, count] : msg_id_counts) {
            //         std::cout << "  Message ID " << msg_id << ": " << count << " times" << std::endl;
            //     }
            //     std::cout << "======================================" << std::endl;
            // }
            
            // Handle specific message types
            switch (message.msgid) {
                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
                    // std::cout << ">>> Processing ATTITUDE_QUATERNION message (ID: " << message.msgid << ")" << std::endl;
                    mavlink_attitude_quaternion_t att_quat;
                    mavlink_msg_attitude_quaternion_decode(&message, &att_quat);
                    
                    // Store quaternion data with thread safety
                    {
                        std::lock_guard<std::mutex> lock(quaternion_mutex_);
                        q1_ = att_quat.q1;  // w component (real part)
                        q2_ = att_quat.q2;  // x component
                        q3_ = att_quat.q3;  // y component
                        q4_ = att_quat.q4;  // z component
                        quaternion_timestamp_ = att_quat.time_boot_ms * 1000; // Convert to microseconds
                        quaternion_valid_ = true;
                    }
                    // Log the received quaternion for debugging
                    UAV::logger().Write("ATTQ", "TimeUs,QuatW,QuatX,QuatY,QuatZ", "Qffff",
                                       UAV::logger().getMicroseconds(),
                                       q1_, q2_, q3_, q4_);
                    // std::cout << ">>> Stored quaternion: w=" << att_quat.q1 
                    //           << " x=" << att_quat.q2 
                    //           << " y=" << att_quat.q3 
                    //           << " z=" << att_quat.q4 << std::endl;
                    break;
                }
                // case MAVLINK_MSG_ID_ATTITUDE: {
                //     std::cout << ">>> Processing ATTITUDE message (ID: " << message.msgid << ")" << std::endl;
                //     // We're not converting attitude to quaternion anymore
                //     // Just log that we received it for debugging
                //     mavlink_attitude_t att;
                //     mavlink_msg_attitude_decode(&message, &att);
                //     std::cout << ">>> Received ATTITUDE: roll=" << att.roll 
                //               << " pitch=" << att.pitch 
                //               << " yaw=" << att.yaw << std::endl;
                //     break;
                // }
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

// Get attitude quaternion
bool MavlinkCommModule::getAttitudeQuaternion(float& q1, float& q2, float& q3, float& q4) const {
    std::lock_guard<std::mutex> lock(quaternion_mutex_);
    if (quaternion_valid_) {
        q1 = q1_;
        q2 = q2_;
        q3 = q3_;
        q4 = q4_;
        return true;
    }
    return false;
}

// Request ATTITUDE_QUATERNION messages from the vehicle
bool MavlinkCommModule::requestAttitudeQuaternion(uint32_t interval_us) {
    std::cout << "=== requestAttitudeQuaternion() called ===" << std::endl;
    std::cout << "Requesting ATTITUDE_QUATERNION messages with interval " << interval_us << " microseconds" << std::endl;
    
    // Debug port status
    std::cout << "Port status: port_=" << (port_ ? "valid" : "null") 
              << ", is_running=" << (port_ && port_->is_running() ? "true" : "false") << std::endl;
    
    // Create MAV_CMD_SET_MESSAGE_INTERVAL command
    mavlink_message_t msg;
    mavlink_command_long_t cmd = {};
    
    cmd.target_system = target_system_id_;
    cmd.target_component = target_component_id_;
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;  // Command ID 511
    cmd.confirmation = 0;
    cmd.param1 = 31;  // ATTITUDE_QUATERNION message ID
    cmd.param2 = interval_us;  // Interval in microseconds
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;
    
    std::cout << "Command details: target_sys=" << (int)cmd.target_system 
              << ", target_comp=" << (int)cmd.target_component 
              << ", command=" << cmd.command 
              << ", param1=" << cmd.param1 
              << ", param2=" << cmd.param2 << std::endl;
    
    // Encode the message
    mavlink_msg_command_long_encode(system_id_, component_id_, &msg, &cmd);
    
    std::cout << "Message encoded, attempting to send..." << std::endl;
    
    // Send the message
    if (port_ && port_->is_running()) {
        int result = port_->write_message(msg);
        if (result > 0) {
            std::cout << "Successfully sent ATTITUDE_QUATERNION request" << std::endl;
            return true;
        } else {
            std::cerr << "Failed to send ATTITUDE_QUATERNION request" << std::endl;
            return false;
        }
    } else {
        std::cerr << "Cannot send ATTITUDE_QUATERNION request: port not running" << std::endl;
        return false;
    }
}

// Get quaternion timestamp
uint64_t MavlinkCommModule::getQuaternionTimestamp() const {
    std::lock_guard<std::mutex> lock(quaternion_mutex_);
    return quaternion_timestamp_;
}