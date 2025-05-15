#include "CameraSource.h"
#include "GazeboSource.h"
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

// src/CameraSource.cpp - Add these implementations

void CameraSource::printCameraModes() {
    if (!capture.isOpened()) {
        std::cerr << "Camera not opened, cannot query modes" << std::endl;
        return;
    }
    
    std::cout << "Camera Properties:" << std::endl;
    std::cout << "---------------" << std::endl;
    
    // Common properties to check
    std::vector<std::pair<int, std::string>> props = {
        {cv::CAP_PROP_FRAME_WIDTH, "Width"},
        {cv::CAP_PROP_FRAME_HEIGHT, "Height"},
        {cv::CAP_PROP_FPS, "FPS"},
        {cv::CAP_PROP_FOURCC, "FourCC"},
        {cv::CAP_PROP_FORMAT, "Format"},
        {cv::CAP_PROP_MODE, "Mode"},
        {cv::CAP_PROP_BUFFERSIZE, "Buffer Size"},
        {cv::CAP_PROP_AUTOFOCUS, "Autofocus"},
        {cv::CAP_PROP_BRIGHTNESS, "Brightness"},
        {cv::CAP_PROP_CONTRAST, "Contrast"},
        {cv::CAP_PROP_SATURATION, "Saturation"},
        {cv::CAP_PROP_HUE, "Hue"},
        {cv::CAP_PROP_GAIN, "Gain"},
        {cv::CAP_PROP_EXPOSURE, "Exposure"}
    };
    
    for (const auto& prop : props) {
        double val = capture.get(prop.first);
        std::cout << prop.second << ": " << val;
        
        // Special handling for FourCC
        if (prop.first == cv::CAP_PROP_FOURCC) {
            int fourcc = static_cast<int>(val);
            char c1 = fourcc & 0xFF;
            char c2 = (fourcc >> 8) & 0xFF;
            char c3 = (fourcc >> 16) & 0xFF;
            char c4 = (fourcc >> 24) & 0xFF;
            std::cout << " (" << c1 << c2 << c3 << c4 << ")";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\nNote: Not all properties are readable from all cameras." << std::endl;
    std::cout << "Use v4l2-ctl --list-formats-ext for more detailed information." << std::endl;
}

bool CameraSource::setBestMode(int targetWidth, int targetHeight, double targetFps) {
    if (!capture.isOpened()) {
        std::cerr << "Camera not opened, cannot set mode" << std::endl;
        return false;
    }
    
    // First, try setting the exact requested values
    bool success = true;
    success &= capture.set(cv::CAP_PROP_FRAME_WIDTH, targetWidth);
    success &= capture.set(cv::CAP_PROP_FRAME_HEIGHT, targetHeight);
    success &= capture.set(cv::CAP_PROP_FPS, targetFps);
    
    // Check what we actually got
    int actualWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actualFps = capture.get(cv::CAP_PROP_FPS);
    
    // If settings didn't apply correctly, we could try common alternatives
    // This is a simplified approach - a more comprehensive implementation would
    // query the camera for all supported modes and pick the closest match
    
    std::cout << "Attempted settings: " << targetWidth << "x" << targetHeight 
              << " @ " << targetFps << " FPS" << std::endl;
    std::cout << "Actual settings: " << actualWidth << "x" << actualHeight 
              << " @ " << actualFps << " FPS" << std::endl;
    
    // Update our stored values to match what the camera is actually doing
    width = actualWidth;
    height = actualHeight;
    fps = actualFps;
    
    return success;
}

// Modify initialize() to use these new methods
bool CameraSource::initialize() {
    // Try to open with specified API first
    capture.open(cameraId, apiPreference);
    
    // Fall back to default API if specified one fails
    if (!capture.isOpened() && apiPreference != 0) {
        std::cout << "Trying to open camera with default API..." << std::endl;
        capture.open(cameraId);
    }
    
    if (!capture.isOpened()) {
        std::cerr << "Error: Could not open camera with ID " << cameraId << std::endl;
        return false;
    }
    
    // Print current camera capabilities before setting anything
    std::cout << "Camera capabilities before configuration:" << std::endl;
    printCameraModes();
    
    // Set camera properties
    std::cout << "\nAttempting to configure camera..." << std::endl;
    setBestMode(width, height, fps);
    
    // Set preferred pixel format (MJPG is often more efficient than YUYV)
    capture.set(cv::CAP_PROP_FOURCC, fourcc);
    
    // Verify camera works by reading a test frame
    cv::Mat testFrame;
    if (!capture.read(testFrame) || testFrame.empty()) {
        std::cerr << "Error: Camera failed to provide a valid test frame" << std::endl;
        capture.release();
        return false;
    }
    
    // Update dimensions based on actual frame (which may differ from what the camera reported)
    width = testFrame.cols;
    height = testFrame.rows;
    
    std::cout << "Camera successfully initialized with resolution: " 
              << width << "x" << height 
              << " @ " << fps << " FPS (reported)" << std::endl;
    
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
