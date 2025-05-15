// src/GazeboSource.cpp
#include "../include/GazeboSource.h"
#include <iostream>

GazeboSource::GazeboSource(const std::string& topicName, int width, int height, double fps)
    : topicName(topicName),
      width(width),
      height(height),
      fps(fps),
      initialized(false),
      hasNewFrame(false),
      running(false) {
}

GazeboSource::~GazeboSource() {
    release();
}

bool GazeboSource::initialize() {
    if (initialized) {
        return true;
    }
    
    std::cout << "Initializing GazeboSource with topic: " << topicName << std::endl;
    
    // Subscribe to the Gazebo topic
    if (!node.Subscribe(topicName, &GazeboSource::onImage, this)) {
        std::cerr << "Error: Failed to subscribe to Gazebo topic: " << topicName << std::endl;
        return false;
    }
    
    running = true;
    initialized = true;
    lastFrameTime = std::chrono::steady_clock::now();
    std::cout << "GazeboSource initialized successfully" << std::endl;
    
    return true;
}

void GazeboSource::onImage(const gz::msgs::Image& _msg) {
    if (!running) {
        return;
    }
    
    // If width/height were not specified, get them from the first message
    if (width == 0 || height == 0) {
        width = _msg.width();
        height = _msg.height();
        std::cout << "GazeboSource: Detected resolution " << width << "x" << height << std::endl;
    }
    
    // Convert Gazebo image to OpenCV format
    cv::Mat frame;
    
    // Handle different pixel formats
    if (_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
        // Create OpenCV matrix from Gazebo image data
        frame = cv::Mat(_msg.height(), _msg.width(), CV_8UC3, 
                       const_cast<char*>(_msg.data().c_str()));
        
        // Convert RGB to BGR (OpenCV uses BGR)
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    } 
    else {
        std::cerr << "GazeboSource: Unsupported pixel format: " << _msg.pixel_format_type() << std::endl;
        return;
    }
    
    // Store the frame
    std::lock_guard<std::mutex> lock(frameMutex);
    frame.copyTo(latestFrame);
    hasNewFrame = true;
}

bool GazeboSource::isInitialized() const {
    return initialized;
}

bool GazeboSource::getNextFrame(cv::Mat& frame) {
    if (!initialized) {
        return false;
    }
    
    // Frame rate control
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        now - lastFrameTime).count();
    auto frameInterval = static_cast<int64_t>(1000000.0 / fps);
    
    if (elapsed < frameInterval) {
        // Not time for a new frame yet
        std::this_thread::sleep_for(std::chrono::microseconds(frameInterval - elapsed));
    }
    
    // Get the latest frame
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (!latestFrame.empty() && hasNewFrame) {
            latestFrame.copyTo(frame);
            hasNewFrame = false;
            lastFrameTime = std::chrono::steady_clock::now();
            return true;
        }
    }
    
    return false;
}

int GazeboSource::getWidth() const {
    return width;
}

int GazeboSource::getHeight() const {
    return height;
}

double GazeboSource::getFPS() const {
    return fps;
}

void GazeboSource::release() {
    if (initialized) {
        running = false;
        node.Unsubscribe(topicName);
        initialized = false;
        
        // Clear the frame buffer
        std::lock_guard<std::mutex> lock(frameMutex);
        latestFrame.release();
    }
}
