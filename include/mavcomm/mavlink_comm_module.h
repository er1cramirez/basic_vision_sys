/**
 * @file mavlink_comm_module.h
 * @brief MAVLink communication module for sending force control vectors
 * 
 * This module provides a unified interface for MAVLink communication over
 * both UDP and Serial connections with configurable timing and thread safety.
 */

#ifndef MAVLINK_COMM_MODULE_H
#define MAVLINK_COMM_MODULE_H

#include <common/mavlink.h>
#include "ardupilotmega/ardupilotmega.h"
#include "generic_port.h"
#include "udp_port.h"
#include "serial_port.h"
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <vector>
#include <string>
#include <cmath>

/**
 * @struct AttitudeData
 * @brief Structure to hold attitude quaternion data with timestamp and derived values
 */
struct AttitudeData {
    // Quaternion components (w, x, y, z)
    float q1 = 1.0f;  // w component (real part)
    float q2 = 0.0f;  // x component
    float q3 = 0.0f;  // y component
    float q4 = 0.0f;  // z component
    
    // Timestamp information
    uint64_t timestamp_us = 0;     // Message timestamp in microseconds
    uint64_t received_time_us = 0; // Time when message was received
    
    // Validity and statistics
    bool valid = false;
    double frequency_hz = 0.0;
    size_t message_count = 0;
    
    // Derived Euler angles (computed automatically)
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    
    /**
     * @brief Update Euler angles from quaternion
     */
    void updateEulerAngles() {
        // Convert quaternion to Euler angles (ZYX convention)
        roll_deg = atan2(2*(q1*q2 + q3*q4), 1 - 2*(q2*q2 + q3*q3)) * 180.0f / M_PI;
        pitch_deg = asin(std::max(-1.0f, std::min(1.0f, 2*(q1*q3 - q4*q2)))) * 180.0f / M_PI;
        yaw_deg = atan2(2*(q1*q4 + q2*q3), 1 - 2*(q3*q3 + q4*q4)) * 180.0f / M_PI;
    }
    
    /**
     * @brief Get age of the data in milliseconds
     */
    double getAgeMs() const {
        auto now = std::chrono::steady_clock::now();
        auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
        return (now_us - received_time_us) / 1000.0;
    }
};

/**
 * @class MavlinkCommModule
 * @brief Class for handling MAVLink communication with configurable frequency
 */
class MavlinkCommModule {
public:
    /**
     * @brief Constructor for UDP communication
     * @param target_ip IP address for UDP communication
     * @param udp_port UDP port number
     * @param system_id MAVLink system ID for this system
     * @param component_id MAVLink component ID for this system
     * @param target_system_id Target system ID
     * @param target_component_id Target component ID
     */
    MavlinkCommModule(const std::string& target_ip, int udp_port, 
                      uint8_t system_id = 255, uint8_t component_id = 1,
                      uint8_t target_system_id = 1, uint8_t target_component_id = 0);
    
    /**
     * @brief Constructor for Serial communication
     * @param uart_name Serial port name
     * @param baudrate Serial port baudrate
     * @param system_id MAVLink system ID for this system
     * @param component_id MAVLink component ID for this system
     * @param target_system_id Target system ID
     * @param target_component_id Target component ID
     * @param is_serial_flag Flag to differentiate from UDP constructor (always true)
     */
    MavlinkCommModule(const std::string& uart_name, int baudrate,
                      uint8_t system_id, uint8_t component_id,
                      uint8_t target_system_id, uint8_t target_component_id,
                      bool is_serial_flag);
    
    /**
     * @brief Destructor
     */
    ~MavlinkCommModule();
    
    /**
     * @brief Start communication
     * @return True if started successfully
     */
    bool start();
    
    /**
     * @brief Stop communication
     */
    void stop();
    
    /**
     * @brief Check if communication is running
     * @return True if running
     */
    bool isRunning() const;
    
    /**
     * @brief Set the force vector sending frequency
     * @param frequency_hz Frequency in Hz
     */
    void setFrequency(double frequency_hz);
    
    /**
     * @brief Get the current sending frequency
     * @return Current frequency in Hz
     */
    double getFrequency() const;
    
    /**
     * @brief Set force vector
     * @param force_x X component of force vector
     * @param force_y Y component of force vector
     * @param force_z Z component of force vector
     * @param force_derivative_x X component of force derivative
     * @param force_derivative_y Y component of force derivative
     * @param force_derivative_z Z component of force derivative
     */
    void setForceVector(float force_x, float force_y, float force_z,
                        float force_derivative_x = 0.0f, 
                        float force_derivative_y = 0.0f,
                        float force_derivative_z = 0.0f);
    
    /**
     * @brief Send the current force vector immediately (non-scheduled)
     * @return True if message was sent successfully
     */
    bool sendForceVectorNow();
    
    /**
     * @brief Register a callback for received MAVLink messages
     * @param callback Callback function that takes a mavlink_message_t
     */
    void registerMessageCallback(std::function<void(const mavlink_message_t&)> callback);
    
    // NOTE: Quaternion/attitude methods removed - now handled by SimpleQuaternionReader
    
private:
    // Communication parameters
    std::unique_ptr<Generic_Port> port_;
    bool using_udp_;
    uint8_t system_id_;
    uint8_t component_id_;
    uint8_t target_system_id_;
    uint8_t target_component_id_;
    
    // Force vector data
    std::mutex force_vector_mutex_;
    float force_x_ = 0.0f;
    float force_y_ = 0.0f;
    float force_z_ = 0.0f;
    float force_derivative_x_ = 0.0f;
    float force_derivative_y_ = 0.0f;
    float force_derivative_z_ = 0.0f;
    uint64_t msg_seq_ = 0;
    
    // NOTE: Quaternion data members removed - now handled by SimpleQuaternionReader
    
    // Thread control
    std::atomic<bool> running_{false};
    std::thread send_thread_;
    std::thread receive_thread_;
    
    // Timing control
    mutable std::mutex frequency_mutex_;  // Mutable to allow locking in const methods
    double frequency_hz_ = 10.0; // Default 10Hz
    std::chrono::microseconds period_{100000}; // 10Hz period in microseconds
    
    // Message callback
    std::mutex callback_mutex_;
    std::function<void(const mavlink_message_t&)> message_callback_;
    
    // Thread functions
    void sendThreadFunc();
    void receiveThreadFunc();
    
    // Update the sending period based on frequency
    void updatePeriod();
    
    // Compose a FORCE_VECTOR_TARGET message
    mavlink_message_t composeForceVectorMessage();
    
    // NOTE: processAttitudeMessage removed - quaternion processing now handled by SimpleQuaternionReader
};

#endif // MAVLINK_COMM_MODULE_H