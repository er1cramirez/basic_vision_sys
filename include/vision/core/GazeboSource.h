// include/GazeboSource.h
#ifndef GAZEBO_SOURCE_H
#define GAZEBO_SOURCE_H

#include "ImageSourceInterface.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

/**
 * @class GazeboSource
 * @brief Provides frames directly from Gazebo simulation
 * 
 * Uses Gazebo transport to subscribe to camera topics
 */
class GazeboSource : public ImageSourceInterface {
public:
    /**
     * Constructor
     * @param topicName Gazebo topic name for the camera
     * @param width Expected width (0 for auto-detect)
     * @param height Expected height (0 for auto-detect)
     * @param fps Target FPS for frame rate control
     */
    GazeboSource(const std::string& topicName = "/world/map/model/iris/link/camera_link/sensor/camera/image",
                int width = 0,
                int height = 0,
                double fps = 30.0);
    
    /**
     * Destructor
     */
    ~GazeboSource() override;
    
    // ImageSourceInterface implementation
    bool initialize() override;
    bool isInitialized() const override;
    bool getNextFrame(cv::Mat& frame) override;
    int getWidth() const override;
    int getHeight() const override;
    double getFPS() const override;
    void release() override;
    
private:
    // Callback for Gazebo image messages
    void onImage(const gz::msgs::Image& _msg);
    
    std::string topicName;       // Gazebo topic name
    int width;                   // Frame width
    int height;                  // Frame height
    double fps;                  // Target FPS
    bool initialized;            // Initialization flag
    std::atomic<bool> hasNewFrame; // Flag indicating new frame is available
    
    // Gazebo transport
    gz::transport::Node node;
    
    // Frame buffer and synchronization
    cv::Mat latestFrame;
    std::mutex frameMutex;
    
    // Control flags
    std::atomic<bool> running;
    
    // Frame timing
    std::chrono::steady_clock::time_point lastFrameTime;
};

#endif // GAZEBO_SOURCE_H
