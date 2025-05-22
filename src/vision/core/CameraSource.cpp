#include "CameraSource.h"
#include <iostream>
#include <thread>
#include <chrono>

CameraSource::CameraSource(int cameraId, int width, int height, double fps, int apiPreference, int fourcc)
    : cameraId(cameraId),
      width(width),
      height(height),
      fps(fps),
      apiPreference(apiPreference),
      fourcc(fourcc),
      initialized(false) {
    // Don't initialize capture here - do it in initialize()
}

CameraSource::~CameraSource() {
    release();
}

bool CameraSource::initialize() {
    if (initialized) {
        return true;
    }
    
    std::cout << "Attempting to open camera " << cameraId << " with API " << apiPreference << std::endl;
    
    // Try to open with specified API first
    if (!capture.open(cameraId, apiPreference)) {
        std::cout << "Failed with specified API, trying default API..." << std::endl;
        
        // Fall back to default API if specified one fails
        if (!capture.open(cameraId)) {
            std::cerr << "Error: Could not open camera with ID " << cameraId << std::endl;
            return false;
        }
    }
    
    if (!capture.isOpened()) {
        std::cerr << "Error: Camera is not opened after initialization" << std::endl;
        return false;
    }
    
    std::cout << "Camera opened successfully" << std::endl;
    
    // Set buffer size to reduce latency (important for real-time applications)
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    // Configure camera properties in the right order
    bool configSuccess = true;
    
    // Set frame format first (this can affect available resolutions)
    if (fourcc != 0) {
        if (!capture.set(cv::CAP_PROP_FOURCC, fourcc)) {
            std::cout << "Warning: Could not set desired FOURCC format" << std::endl;
        }
    }
    
    // Set resolution
    if (!capture.set(cv::CAP_PROP_FRAME_WIDTH, width)) {
        std::cout << "Warning: Could not set desired width: " << width << std::endl;
        configSuccess = false;
    }
    
    if (!capture.set(cv::CAP_PROP_FRAME_HEIGHT, height)) {
        std::cout << "Warning: Could not set desired height: " << height << std::endl;
        configSuccess = false;
    }
    
    // Set FPS
    if (!capture.set(cv::CAP_PROP_FPS, fps)) {
        std::cout << "Warning: Could not set desired FPS: " << fps << std::endl;
        configSuccess = false;
    }
    
    // Get actual values that were set
    int actualWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actualFps = capture.get(cv::CAP_PROP_FPS);
    
    std::cout << "Camera configuration:" << std::endl;
    std::cout << "  Requested: " << width << "x" << height << " @ " << fps << " FPS" << std::endl;
    std::cout << "  Actual: " << actualWidth << "x" << actualHeight << " @ " << actualFps << " FPS" << std::endl;
    
    // Update our stored values to match reality
    width = actualWidth;
    height = actualHeight;
    fps = actualFps;
    
    // Test frame capture before declaring success
    cv::Mat testFrame;
    bool testCapture = false;
    
    // Try a few times to get a test frame
    for (int i = 0; i < 5 && !testCapture; i++) {
        if (capture.read(testFrame) && !testFrame.empty()) {
            testCapture = true;
            
            // Update dimensions from actual frame (most reliable method)
            width = testFrame.cols;
            height = testFrame.rows;
            
            std::cout << "Test frame captured successfully: " << width << "x" << height << std::endl;
        } else {
            std::cout << "Test frame " << (i + 1) << " failed, retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    if (!testCapture) {
        std::cerr << "Error: Camera failed to provide valid test frames" << std::endl;
        capture.release();
        return false;
    }
    
    // Print final camera properties for debugging
    printCameraModes();
    
    initialized = true;
    std::cout << "Camera initialization completed successfully" << std::endl;
    return true;
}

void CameraSource::printCameraModes() {
    if (!capture.isOpened()) {
        std::cerr << "Camera not opened, cannot query modes" << std::endl;
        return;
    }
    
    std::cout << "\nCamera Properties:" << std::endl;
    std::cout << "==================" << std::endl;
    
    // Safe property reading with error checking
    auto safeGet = [this](int prop, const std::string& name) -> double {
        try {
            double val = capture.get(prop);
            std::cout << name << ": " << val;
            
            // Special handling for FourCC
            if (prop == cv::CAP_PROP_FOURCC && val > 0) {
                int fourccVal = static_cast<int>(val);
                char c1 = fourccVal & 0xFF;
                char c2 = (fourccVal >> 8) & 0xFF;
                char c3 = (fourccVal >> 16) & 0xFF;
                char c4 = (fourccVal >> 24) & 0xFF;
                // Only print if characters are printable
                if (c1 >= 32 && c1 <= 126 && c2 >= 32 && c2 <= 126 && 
                    c3 >= 32 && c3 <= 126 && c4 >= 32 && c4 <= 126) {
                    std::cout << " (" << c1 << c2 << c3 << c4 << ")";
                }
            }
            std::cout << std::endl;
            return val;
        } catch (...) {
            std::cout << name << ": (unavailable)" << std::endl;
            return -1;
        }
    };
    
    safeGet(cv::CAP_PROP_FRAME_WIDTH, "Width");
    safeGet(cv::CAP_PROP_FRAME_HEIGHT, "Height");
    safeGet(cv::CAP_PROP_FPS, "FPS");
    safeGet(cv::CAP_PROP_FOURCC, "FourCC");
    safeGet(cv::CAP_PROP_FORMAT, "Format");
    safeGet(cv::CAP_PROP_BUFFERSIZE, "Buffer Size");
    safeGet(cv::CAP_PROP_BRIGHTNESS, "Brightness");
    safeGet(cv::CAP_PROP_CONTRAST, "Contrast");
    safeGet(cv::CAP_PROP_SATURATION, "Saturation");
    safeGet(cv::CAP_PROP_EXPOSURE, "Exposure");
    safeGet(cv::CAP_PROP_GAIN, "Gain");
    
    std::cout << "==================" << std::endl;
}

bool CameraSource::setBestMode(int targetWidth, int targetHeight, double targetFps) {
    if (!capture.isOpened()) {
        std::cerr << "Camera not opened, cannot set mode" << std::endl;
        return false;
    }
    
    std::cout << "Setting camera mode: " << targetWidth << "x" << targetHeight 
              << " @ " << targetFps << " FPS" << std::endl;
    
    bool success = true;
    
    // Set properties one by one with error checking
    if (!capture.set(cv::CAP_PROP_FRAME_WIDTH, targetWidth)) {
        std::cout << "Warning: Failed to set width to " << targetWidth << std::endl;
        success = false;
    }
    
    if (!capture.set(cv::CAP_PROP_FRAME_HEIGHT, targetHeight)) {
        std::cout << "Warning: Failed to set height to " << targetHeight << std::endl;
        success = false;
    }
    
    if (!capture.set(cv::CAP_PROP_FPS, targetFps)) {
        std::cout << "Warning: Failed to set FPS to " << targetFps << std::endl;
        success = false;
    }
    
    // Verify what we actually got
    int actualWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actualFps = capture.get(cv::CAP_PROP_FPS);
    
    std::cout << "Mode set result - Actual: " << actualWidth << "x" << actualHeight 
              << " @ " << actualFps << " FPS" << std::endl;
    
    // Update our stored values
    width = actualWidth;
    height = actualHeight;
    fps = actualFps;
    
    return success;
}

bool CameraSource::isInitialized() const {
    return initialized && capture.isOpened();
}

bool CameraSource::getNextFrame(cv::Mat& frame) {
    if (!isInitialized()) {
        std::cerr << "Camera not initialized" << std::endl;
        return false;
    }
    
    std::lock_guard<std::mutex> lock(captureMutex);
    
    // Clear any old data from the frame
    frame.release();
    
    // Grab and retrieve in one step for better performance and thread safety
    if (!capture.read(frame)) {
        std::cerr << "Failed to read frame from camera" << std::endl;
        return false;
    }
    
    if (frame.empty()) {
        std::cerr << "Captured frame is empty" << std::endl;
        return false;
    }
    
    // Verify frame dimensions match expectations
    if (frame.cols != width || frame.rows != height) {
        std::cout << "Frame size mismatch - Expected: " << width << "x" << height 
                  << ", Got: " << frame.cols << "x" << frame.rows << std::endl;
        // Update our stored dimensions (this is not thread-safe, but it's informational)
        width = frame.cols;
        height = frame.rows;
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
    std::lock_guard<std::mutex> lock(captureMutex);
    if (capture.isOpened()) {
        std::cout << "Releasing camera resources" << std::endl;
        capture.release();
    }
    initialized = false;
}

bool CameraSource::setExposure(int value) {
    return safeSetProperty(cv::CAP_PROP_EXPOSURE, value);
}

bool CameraSource::setAutoExposure(bool enable) {
    return safeSetProperty(cv::CAP_PROP_AUTO_EXPOSURE, enable ? 1 : 0);
}

double CameraSource::safeGetProperty(int prop) const {
    std::lock_guard<std::mutex> lock(captureMutex);
    if (!capture.isOpened()) {
        return -1;
    }
    
    try {
        return capture.get(prop);
    } catch (...) {
        return -1;
    }
}

bool CameraSource::safeSetProperty(int prop, double value) {
    std::lock_guard<std::mutex> lock(captureMutex);
    if (!capture.isOpened()) {
        return false;
    }
    
    try {
        return capture.set(prop, value);
    } catch (...) {
        return false;
    }
}