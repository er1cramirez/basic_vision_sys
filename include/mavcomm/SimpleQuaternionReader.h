/**
 * @file SimpleQuaternionReader.h
 * @brief Simple, high-frequency quaternion reader for MAVProxy output
 */

#ifndef SIMPLE_QUATERNION_READER_H
#define SIMPLE_QUATERNION_READER_H

#include <common/mavlink.h>
#include "ardupilotmega/ardupilotmega.h"
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

/**
 * @struct SimpleQuaternionData
 * @brief Lightweight quaternion data structure
 */
struct SimpleQuaternionData {
    float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;
    uint64_t timestamp_us = 0;
    std::chrono::steady_clock::time_point received_time;
    bool valid = false;
    size_t message_count = 0;
    double frequency_hz = 0.0;
    
    double getAgeMs() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - received_time).count();
    }
};

/**
 * @class SimpleQuaternionReader
 * @brief Simple, dedicated quaternion reader for reliable 50Hz operation
 */
class SimpleQuaternionReader {
public:
    SimpleQuaternionReader(int port = 14552);
    ~SimpleQuaternionReader();
    
    bool start();
    void stop();
    bool getLatestData(SimpleQuaternionData& data) const;
    double getFrequency() const;
    size_t getMessageCount() const;
    bool isRunning() const { return running_; }

private:
    int port_;
    int sock_ = -1;
    std::thread receive_thread_;
    std::atomic<bool> running_{false};
    
    mutable std::mutex data_mutex_;
    SimpleQuaternionData latest_data_;
    std::atomic<size_t> total_messages_{0};
    
    void receiveLoop();
    bool setupSocket();
    void updateFrequency();
    
    // Frequency calculation
    std::chrono::steady_clock::time_point last_message_time_;
    double current_frequency_ = 0.0;
};

#endif // SIMPLE_QUATERNION_READER_H
