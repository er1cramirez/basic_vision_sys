#ifndef GENERIC_DEBUG_VISUALIZER_H
#define GENERIC_DEBUG_VISUALIZER_H

#define CPPHTTPLIB_OPENSSL_SUPPORT
#include "httplib.h"

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <iostream>

/**
 * @class GenericDebugVisualizer
 * @brief Generic HTTP-based debug visualizer for computer vision applications
 * 
 * Features:
 * - Live video streaming via HTTP
 * - Generic key-value data display
 * - No dependencies on specific pipeline types
 * - Lightweight and reusable
 * - Modern web interface
 */
class GenericDebugVisualizer {
public:
    /**
     * @brief Constructor
     * @param port HTTP server port (default: 8888)
     * @param title Optional title for the web interface
     */
    GenericDebugVisualizer(int port = 8888, const std::string& title = "Debug Visualizer");
    
    /**
     * @brief Destructor
     */
    ~GenericDebugVisualizer();
    
    /**
     * @brief Start the HTTP server
     */
    void start();
    
    /**
     * @brief Stop the HTTP server
     */
    void stop();
    
    /**
     * @brief Check if server is running
     * @return True if running
     */
    bool isRunning() const;
    
    /**
     * @brief Update the main video frame
     * @param frame OpenCV frame to display
     * @param frameId Optional frame ID for tracking
     */
    void updateFrame(const cv::Mat& frame, int frameId = -1);
    
    /**
     * @brief Set a data value (string)
     * @param key Data key/name
     * @param value String value
     */
    void setData(const std::string& key, const std::string& value);
    
    /**
     * @brief Set a data value (numeric)
     * @param key Data key/name
     * @param value Numeric value
     */
    void setData(const std::string& key, double value);
    
    /**
     * @brief Set a data value (integer)
     * @param key Data key/name
     * @param value Integer value
     */
    void setData(const std::string& key, int value);
    
    /**
     * @brief Set a data value (boolean)
     * @param key Data key/name
     * @param value Boolean value
     */
    void setData(const std::string& key, bool value);
    
    /**
     * @brief Set multiple data values at once
     * @param data Map of key-value pairs (strings)
     */
    void setData(const std::map<std::string, std::string>& data);
    
    /**
     * @brief Remove a data entry
     * @param key Data key to remove
     */
    void removeData(const std::string& key);
    
    /**
     * @brief Clear all data
     */
    void clearData();
    
    /**
     * @brief Set system status message
     * @param status Status message
     */
    void setStatus(const std::string& status);
    
    /**
     * @brief Get server port
     * @return Port number
     */
    int getPort() const;
    
    /**
     * @brief Set JPEG quality for video streaming (1-100)
     * @param quality JPEG quality level
     */
    void setJpegQuality(int quality);
    
    /**
     * @brief Set maximum frame dimensions for streaming
     * @param width Maximum width
     * @param height Maximum height
     */
    void setMaxFrameSize(int width, int height);
    
    /**
     * @brief Set interface title
     * @param title New title for web interface
     */
    void setTitle(const std::string& title);

private:
    // Configuration
    int serverPort;
    std::string interfaceTitle;
    int jpegQuality;
    int maxFrameWidth;
    int maxFrameHeight;
    int maxQueueSize;
    
    // Threading
    std::thread serverThread;
    std::atomic<bool> running;
    
    // Frame management
    std::queue<cv::Mat> frameQueue;
    std::mutex frameMutex;
    std::condition_variable frameCondition;
    
    // Current frame data
    std::mutex currentFrameMutex;
    std::vector<uchar> currentJpeg;
    std::atomic<uint64_t> frameCounter{0};
    int lastFrameId;
    
    // Data storage
    std::mutex dataMutex;
    std::map<std::string, std::string> dataMap;
    std::string systemStatus;
    
    // Performance tracking
    std::chrono::steady_clock::time_point startTime;
    std::chrono::steady_clock::time_point lastFrameTime;
    double currentFps;
    
    // Server functions
    void serverLoop();
    std::string generateHtmlPage();
    std::string generateDataJson();
    std::string generateSystemInfo();
    
    // Frame processing
    void processFrameQueue();
    void updateFpsCounter();
};

#endif // GENERIC_DEBUG_VISUALIZER_H