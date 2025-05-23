#ifndef VISUALIZATION_DATA_H
#define VISUALIZATION_DATA_H

#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <chrono>

/**
 * @struct VisualizationData
 * @brief Container for all data needed by the visualization system
 */
struct VisualizationData {
    // Frame data
    cv::Mat debugImage;
    int frameId;
    
    // System status
    std::map<std::string, std::string> statusData;
    
    // Performance metrics
    double visionFps;
    double ekfFps;
    double controlFps;
    
    // Timestamp
    std::chrono::steady_clock::time_point timestamp;
    
    // Constructor
    VisualizationData() : frameId(-1), visionFps(0), ekfFps(0), controlFps(0) {
        timestamp = std::chrono::steady_clock::now();
    }
    
    // Helper to add/update status data
    void setStatus(const std::string& key, const std::string& value) {
        statusData[key] = value;
    }
    
    void setStatus(const std::string& key, double value) {
        statusData[key] = std::to_string(value);
    }
    
    void setStatus(const std::string& key, int value) {
        statusData[key] = std::to_string(value);
    }
};

#endif // VISUALIZATION_DATA_H