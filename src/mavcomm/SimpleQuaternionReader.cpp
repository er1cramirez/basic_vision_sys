/**
 * @file SimpleQuaternionReader.cpp
 * @brief Simple, high-frequency quaternion reader implementation
 */

#include "SimpleQuaternionReader.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

SimpleQuaternionReader::SimpleQuaternionReader(int port) : port_(port) {
    std::cout << "[SIMPLE_QUAT] Created for port " << port_ << std::endl;
}

SimpleQuaternionReader::~SimpleQuaternionReader() {
    stop();
    if (sock_ >= 0) {
        close(sock_);
    }
}

bool SimpleQuaternionReader::setupSocket() {
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        std::cerr << "[SIMPLE_QUAT] Failed to create socket" << std::endl;
        return false;
    }
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[SIMPLE_QUAT] Failed to bind to port " << port_ << std::endl;
        std::cerr << "[SIMPLE_QUAT] Run 'output add 127.0.0.1:" << port_ << "' in MAVProxy" << std::endl;
        close(sock_);
        sock_ = -1;
        return false;
    }
    
    // Non-blocking
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
    
    std::cout << "[SIMPLE_QUAT] Socket bound to port " << port_ << std::endl;
    return true;
}

bool SimpleQuaternionReader::start() {
    if (running_) {
        return true;
    }
    
    if (!setupSocket()) {
        return false;
    }
    
    running_ = true;
    receive_thread_ = std::thread(&SimpleQuaternionReader::receiveLoop, this);
    
    std::cout << "[SIMPLE_QUAT] ✓ Started - expecting 50Hz quaternion data" << std::endl;
    return true;
}

void SimpleQuaternionReader::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    
    std::cout << "[SIMPLE_QUAT] ✓ Stopped" << std::endl;
}

void SimpleQuaternionReader::receiveLoop() {
    mavlink_message_t message;
    mavlink_status_t status;
    uint8_t buffer[2048];
    
    auto last_stats = std::chrono::steady_clock::now();
    
    std::cout << "[SIMPLE_QUAT] High-frequency receive loop started" << std::endl;
    
    while (running_) {
        ssize_t bytes = recv(sock_, buffer, sizeof(buffer), 0);
        
        if (bytes > 0) {
            for (ssize_t i = 0; i < bytes; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status)) {
                    if (message.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
                        mavlink_attitude_quaternion_t quat;
                        mavlink_msg_attitude_quaternion_decode(&message, &quat);
                        
                        auto now = std::chrono::steady_clock::now();
                        
                        {
                            std::lock_guard<std::mutex> lock(data_mutex_);
                            latest_data_.q1 = quat.q1;
                            latest_data_.q2 = quat.q2;
                            latest_data_.q3 = quat.q3;
                            latest_data_.q4 = quat.q4;
                            latest_data_.timestamp_us = quat.time_boot_ms * 1000;
                            latest_data_.received_time = now;
                            latest_data_.valid = true;
                            latest_data_.message_count = ++total_messages_;
                            
                            updateFrequency();
                            latest_data_.frequency_hz = current_frequency_;
                        }
                    }
                }
            }
        } else if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "[SIMPLE_QUAT] Receive error: " << strerror(errno) << std::endl;
            break;
        }
        
        // Stats every 5 seconds
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_stats).count() >= 5.0) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            std::cout << "[SIMPLE_QUAT] Stats: msgs=" << total_messages_ 
                      << ", freq=" << std::fixed << std::setprecision(1) << current_frequency_ << "Hz"
                      << ", age=" << std::setprecision(1) << latest_data_.getAgeMs() << "ms"
                      << std::endl;
            last_stats = now;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    std::cout << "[SIMPLE_QUAT] Receive loop ended" << std::endl;
}

void SimpleQuaternionReader::updateFrequency() {
    auto now = std::chrono::steady_clock::now();
    
    if (total_messages_ > 1) {
        auto interval = std::chrono::duration<double>(now - last_message_time_).count();
        if (interval > 0) {
            double instant_freq = 1.0 / interval;
            // Simple exponential moving average
            current_frequency_ = (current_frequency_ * 0.9) + (instant_freq * 0.1);
        }
    }
    
    last_message_time_ = now;
}

bool SimpleQuaternionReader::getLatestData(SimpleQuaternionData& data) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_data_.valid) {
        return false;
    }
    data = latest_data_;
    return true;
}

double SimpleQuaternionReader::getFrequency() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_frequency_;
}

size_t SimpleQuaternionReader::getMessageCount() const {
    return total_messages_;
}
