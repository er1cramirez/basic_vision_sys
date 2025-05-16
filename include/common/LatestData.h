#ifndef LATEST_DATA_H
#define LATEST_DATA_H

#include <mutex>
#include <atomic>
#include <chrono>

/**
 * @class LatestData
 * @brief Thread-safe container for the latest data of a specific type
 * 
 * This class provides a thread-safe way to share the latest data between
 * different threads. It stores the data along with a timestamp.
 */
template <typename T>
class LatestData {
public:
    /**
     * @brief Constructor
     */
    LatestData() : hasData(false) {}
    
    /**
     * @brief Update the stored data
     * @param newData New data to store
     */
    void update(const T& newData) {
        std::lock_guard<std::mutex> lock(mutex);
        data = newData;
        timestamp = std::chrono::steady_clock::now();
        hasData = true;
    }
    
    /**
     * @brief Get the stored data
     * @param output Reference to store the data
     * @param time Reference to store the timestamp
     * @return True if data was available, false otherwise
     */
    bool getData(T& output, std::chrono::steady_clock::time_point& time) const {
        std::lock_guard<std::mutex> lock(mutex);
        if (!hasData) return false;
        output = data;
        time = timestamp;
        return true;
    }
    
    /**
     * @brief Get just the data without the timestamp
     * @param output Reference to store the data
     * @return True if data was available, false otherwise
     */
    bool getData(T& output) const {
        std::chrono::steady_clock::time_point time;
        return getData(output, time);
    }
    
    /**
     * @brief Check if the data is available
     * @return True if data is available
     */
    bool isDataAvailable() const {
        return hasData;
    }
    
    /**
     * @brief Get the time since last update
     * @return Time in milliseconds since last update
     */
    int64_t timeSinceUpdate() const {
        if (!hasData) return -1;
        
        std::lock_guard<std::mutex> lock(mutex);
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            now - timestamp).count();
    }
    
private:
    T data;
    std::chrono::steady_clock::time_point timestamp;
    mutable std::mutex mutex;
    std::atomic<bool> hasData;
};

#endif // LATEST_DATA_H