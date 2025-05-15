// src/CameraSource.cpp
#include "../include/CameraSource.h"
#include <iostream>

CameraSource::CameraSource(int cameraId, int width, int height, double fps, int apiPreference, int fourcc)
    : capture(), 
      fourcc(fourcc), 
      initialized(false),
      cameraId(cameraId),
      width(width),
      height(height),
      fps(fps),
      apiPreference(apiPreference) {}

CameraSource::~CameraSource() {
    release();
}

bool CameraSource::initialize() {
    // Open the camera with specified API
    capture.open(cameraId, apiPreference);
    
    if (!capture.isOpened()) {
        std::cerr << "Error: Could not open camera with ID " << cameraId << std::endl;
        return false;
    }
    
    // Set camera properties
    capture.set(cv::CAP_PROP_FOURCC, fourcc);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    capture.set(cv::CAP_PROP_FPS, fps);
    
    // Read back actual properties (may differ from requested)
    width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    fps = capture.get(cv::CAP_PROP_FPS);
    
    std::cout << "Camera initialized with resolution: " 
              << width << "x" << height 
              << " @ " << fps << " FPS" << std::endl;
    
    initialized = true;
    return true;
}

bool CameraSource::isInitialized() const {
    return initialized && capture.isOpened();
}

bool CameraSource::getNextFrame(cv::Mat& frame) {
    if (!isInitialized()) {
        return false;
    }
    
    bool success = capture.read(frame);
    
    if (!success || frame.empty()) {
        std::cerr << "Error: Failed to capture frame from camera" << std::endl;
        return false;
    }
    
    return true;
}

int CameraSource::getWidth() const {
    return width;
}

int CameraSource::getHeight() const {
    return height;
}

double CameraSource::getFPS() const {
    return fps;
}

void CameraSource::release() {
    if (capture.isOpened()) {
        capture.release();
        initialized = false;
    }
}
