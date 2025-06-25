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
    
    /**
     * @brief Request ATTITUDE_QUATERNION messages from the vehicle
     * @param interval_us Interval between messages in microseconds (default: 100000 = 10Hz)
     * @return True if request was sent successfully
     */
    bool requestAttitudeQuaternion(uint32_t interval_us = 100000);
    
    /**
     * @brief Get the latest attitude quaternion data
     * @param q1 Quaternion component 1 (w)
     * @param q2 Quaternion component 2 (x)
     * @param q3 Quaternion component 3 (y)
     * @param q4 Quaternion component 4 (z)
     * @return True if valid quaternion data is available
     */
    bool getAttitudeQuaternion(float& q1, float& q2, float& q3, float& q4) const;
    
    /**
     * @brief Get the latest attitude quaternion timestamp
     * @return Timestamp in microseconds since Unix epoch
     */
    uint64_t getQuaternionTimestamp() const;
    
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
    
    // Attitude quaternion data
    mutable std::mutex quaternion_mutex_;
    float q1_ = 1.0f;  // w component (real part)
    float q2_ = 0.0f;  // x component
    float q3_ = 0.0f;  // y component  
    float q4_ = 0.0f;  // z component
    uint64_t quaternion_timestamp_ = 0;
    bool quaternion_valid_ = false;
    
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
};

#endif // MAVLINK_COMM_MODULE_H