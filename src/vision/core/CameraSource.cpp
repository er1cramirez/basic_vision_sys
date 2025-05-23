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

// Optimized CameraSource initialization for minimal latency
bool CameraSource::initialize() {
    if (initialized) {
        return true;
    }
    
    std::cout << "Initializing camera with low-latency settings..." << std::endl;
    
    // CRITICAL: Try different APIs in order of preference
    std::vector<int> apiPreferences = {
        cv::CAP_V4L2,      // Linux native (usually fastest)
        cv::CAP_GSTREAMER, // GStreamer (good for RTSP/network)
        cv::CAP_FFMPEG,    // FFmpeg (fallback)
        cv::CAP_ANY        // Last resort
    };
    
    bool opened = false;
    for (int api : apiPreferences) {
        std::cout << "Trying API: " << api << std::endl;
        if (capture.open(cameraId, api)) {
            std::cout << "Successfully opened with API: " << api << std::endl;
            opened = true;
            break;
        }
    }
    
    if (!opened) {
        std::cerr << "Failed to open camera with any API" << std::endl;
        return false;
    }
    
    // CRITICAL SETTINGS FOR LOW LATENCY (apply before resolution/FPS)
    std::cout << "Applying low-latency camera settings..." << std::endl;
    
    // 1. MINIMAL BUFFER - Most important setting
    if (!capture.set(cv::CAP_PROP_BUFFERSIZE, 1)) {
        std::cout << "Warning: Could not set buffer size to 1" << std::endl;
    } else {
        std::cout << "✓ Buffer size set to 1" << std::endl;
    }
    
    // 2. DISABLE AUTO SETTINGS (they cause delays)
    capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25); // Manual exposure
    capture.set(cv::CAP_PROP_AUTOFOCUS, 0);        // Disable autofocus
    capture.set(cv::CAP_PROP_AUTO_WB, 0);          // Disable auto white balance
    
    // 3. FORCE SPECIFIC FORMAT (avoid format negotiation delays)
    if (fourcc != 0) {
        if (capture.set(cv::CAP_PROP_FOURCC, fourcc)) {
            std::cout << "✓ FourCC format set" << std::endl;
        }
    } else {
        // Try MJPEG first (usually fastest for USB cameras)
        int mjpeg = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        if (capture.set(cv::CAP_PROP_FOURCC, mjpeg)) {
            std::cout << "✓ Using MJPEG format" << std::endl;
        }
    }
    
    // 4. SET LOWER RESOLUTION FIRST (reduces USB bandwidth)
    // Start with lower resolution, then increase if needed
    int testWidth = std::min(width, 640);
    int testHeight = std::min(height, 480);
    
    capture.set(cv::CAP_PROP_FRAME_WIDTH, testWidth);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, testHeight);
    capture.set(cv::CAP_PROP_FPS, fps);
    
    // 5. LINUX-SPECIFIC V4L2 OPTIMIZATIONS
    #ifdef __linux__
    capture.set(cv::CAP_PROP_CONVERT_RGB, 0);  // Avoid conversions
    
    // V4L2 specific settings for minimal latency
    // These may not work with all cameras but worth trying
    if (apiPreferences[0] == cv::CAP_V4L2) {
        // Try to set exposure manually to avoid auto-exposure delays
        capture.set(cv::CAP_PROP_EXPOSURE, -6);  // Fast exposure
        capture.set(cv::CAP_PROP_GAIN, 50);      // Manual gain
        std::cout << "✓ Applied V4L2 manual exposure settings" << std::endl;
    }
    #endif
    
    // 6. VERIFY ACTUAL SETTINGS
    int actualWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actualFps = capture.get(cv::CAP_PROP_FPS);
    int actualBuffer = static_cast<int>(capture.get(cv::CAP_PROP_BUFFERSIZE));
    
    std::cout << "Camera settings verification:" << std::endl;
    std::cout << "  Resolution: " << actualWidth << "x" << actualHeight << std::endl;
    std::cout << "  FPS: " << actualFps << std::endl;
    std::cout << "  Buffer size: " << actualBuffer << std::endl;
    
    // Update our stored values
    width = actualWidth;
    height = actualHeight;
    fps = actualFps;
    
    // 7. PERFORMANCE TEST - Measure actual frame acquisition time
    std::cout << "Testing frame acquisition performance..." << std::endl;
    cv::Mat testFrame;
    double totalTime = 0;
    int successCount = 0;
    
    for (int i = 0; i < 10; i++) {
        auto start = std::chrono::steady_clock::now();
        bool success = capture.read(testFrame);
        auto end = std::chrono::steady_clock::now();
        
        if (success && !testFrame.empty()) {
            double frameTime = std::chrono::duration<double, std::milli>(end - start).count();
            totalTime += frameTime;
            successCount++;
            std::cout << "  Test frame " << (i+1) << ": " << frameTime << "ms" << std::endl;
        }
    }
    
    if (successCount > 0) {
        double avgTime = totalTime / successCount;
        std::cout << "Average frame acquisition time: " << avgTime << "ms" << std::endl;
        
        if (avgTime > 20.0) {
            std::cout << "⚠️  WARNING: Frame acquisition is slow (" << avgTime << "ms)" << std::endl;
            std::cout << "   This will limit your maximum FPS to ~" << (1000.0/avgTime) << std::endl;
            std::cout << "   Consider lower resolution or different camera" << std::endl;
        } else {
            std::cout << "✓ Frame acquisition performance is good (" << avgTime << "ms)" << std::endl;
        }
    }
    
    initialized = true;
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