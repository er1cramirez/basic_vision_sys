// src/vision/core/CameraSource.cpp
#include "CameraSource.h"
#include <iostream>
#include <thread>
#include <chrono>

CameraSource::CameraSource(int cameraId, int width, int height, double fps, 
                          int apiPreference, int fourcc)
    : cameraId(cameraId),
      width(width),
      height(height),
      fps(fps),
      apiPreference(apiPreference),
      fourcc(fourcc),
      preprocessType(FramePreprocessType::RAW),
      shouldCopyFrame(false),  // Default to zero-copy for maximum performance
      rotation(0),
      initialized(false) {
}

CameraSource::~CameraSource() {
    release();
}

bool CameraSource::initialize() {
    if (initialized.load()) {
        return true;
    }
    
    std::cout << "Opening camera " << cameraId << " with API " << apiPreference << std::endl;
    
    // Try specified API first, then fallback
    if (!capture.open(cameraId, apiPreference)) {
        std::cout << "Failed with specified API, trying default..." << std::endl;
        if (!capture.open(cameraId)) {
            std::cerr << "Error: Could not open camera " << cameraId << std::endl;
            return false;
        }
    }
    
    if (!capture.isOpened()) {
        return false;
    }
    
    // CRITICAL PERFORMANCE SETTINGS
    // Set buffer size to 1 to get the latest frame (reduces latency)
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    // Set FourCC for optimal performance
    if (fourcc != 0) {
        capture.set(cv::CAP_PROP_FOURCC, fourcc);
    }
    
    // Set resolution
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    
    // Set FPS
    capture.set(cv::CAP_PROP_FPS, fps);
    
    // Get actual values
    int actualWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actualFps = capture.get(cv::CAP_PROP_FPS);
    
    std::cout << "Camera configuration:" << std::endl;
    std::cout << "  Requested: " << width << "x" << height << " @ " << fps << " FPS" << std::endl;
    std::cout << "  Actual: " << actualWidth << "x" << actualHeight << " @ " << actualFps << " FPS" << std::endl;
    
    width = actualWidth;
    height = actualHeight;
    fps = actualFps;
    
    // Test capture with grab/retrieve pattern for better performance understanding
    bool testSuccess = false;
    for (int i = 0; i < 5 && !testSuccess; i++) {
        if (capture.grab()) {
            cv::Mat testFrame;
            if (capture.retrieve(testFrame) && !testFrame.empty()) {
                testSuccess = true;
                std::cout << "Test frame " << i << " captured successfully" << std::endl;
                
                // Pre-allocate cached frame if not copying
                if (!shouldCopyFrame) {
                    cachedFrame = cv::Mat(testFrame.size(), testFrame.type());
                }
            }
        }
        if (!testSuccess) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    
    if (!testSuccess) {
        std::cerr << "Error: Failed to capture test frames" << std::endl;
        capture.release();
        return false;
    }
    
    initialized = true;
    std::cout << "Camera initialized successfully" << std::endl;
    return true;
}

bool CameraSource::isInitialized() const {
    return initialized.load() && capture.isOpened();
}

bool CameraSource::getNextFrame(cv::Mat& frame) {
    if (!isInitialized()) {
        return false;
    }
    
    // PERFORMANCE CRITICAL SECTION - NO MUTEX NEEDED
    // VideoCapture is thread-safe for single reader
    
    // Method 1: Direct read (fastest if camera supports it well)
    bool success = false;
    
    if (shouldCopyFrame) {
        // Standard read with copy
        success = capture.read(frame);
    } else {
        // Optimized zero-copy path when possible
        // Use grab/retrieve pattern for better control
        if (capture.grab()) {
            // Retrieve directly into the output frame to avoid copies
            success = capture.retrieve(frame);
        }
    }
    
    if (!success || frame.empty()) {
        return false;
    }
    
    // Apply preprocessing if needed (in-place when possible)
    if (preprocessType != FramePreprocessType::RAW) {
        preprocessFrame(frame);
    }
    
    // Apply rotation if needed
    if (rotation != 0) {
        applyRotation(frame);
    }
    
    return true;
}

void CameraSource::preprocessFrame(cv::Mat& frame) {
    switch (preprocessType) {
        case FramePreprocessType::GRAYSCALE:
            if (frame.channels() > 1) {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            }
            break;
            
        case FramePreprocessType::HSV:
            if (frame.channels() == 3) {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
            }
            break;
            
        case FramePreprocessType::THRESHOLD:
            if (frame.channels() > 1) {
                cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            }
            cv::adaptiveThreshold(frame, frame, 255, 
                                cv::ADAPTIVE_THRESH_MEAN_C, 
                                cv::THRESH_BINARY_INV, 11, 5);
            break;
            
        case FramePreprocessType::RAW:
        default:
            // No preprocessing
            break;
    }
}

void CameraSource::applyRotation(cv::Mat& frame) {
    if (rotation == 0) return;
    
    // Fast rotation for 90-degree increments
    switch (rotation) {
        case 90:
            cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);
            break;
        case 180:
            cv::rotate(frame, frame, cv::ROTATE_180);
            break;
        case 270:
            cv::rotate(frame, frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            break;
        default:
            // For arbitrary angles (slower)
            cv::Point2f center(frame.cols / 2.0f, frame.rows / 2.0f);
            cv::Mat rotMat = cv::getRotationMatrix2D(center, rotation, 1.0);
            cv::warpAffine(frame, frame, rotMat, frame.size());
            break;
    }
}

void CameraSource::setPreprocessType(FramePreprocessType type) {
    preprocessType = type;
}

void CameraSource::setShouldCopyFrame(bool copy) {
    shouldCopyFrame = copy;
}

void CameraSource::setRotation(int degrees) {
    rotation = degrees;
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
        std::cout << "Releasing camera resources" << std::endl;
        capture.release();
    }
    initialized = false;
}

bool CameraSource::setExposure(int value) {
    if (!capture.isOpened()) return false;
    return capture.set(cv::CAP_PROP_EXPOSURE, value);
}

bool CameraSource::setAutoExposure(bool enable) {
    if (!capture.isOpened()) return false;
    return capture.set(cv::CAP_PROP_AUTO_EXPOSURE, enable ? 1 : 0);
}

void CameraSource::printCameraModes() {
    if (!capture.isOpened()) {
        std::cerr << "Camera not opened" << std::endl;
        return;
    }
    
    std::cout << "\nCamera Properties:" << std::endl;
    std::cout << "==================" << std::endl;
    
    // Helper lambda for safe property reading
    auto printProp = [this](int prop, const std::string& name) {
        double val = capture.get(prop);
        std::cout << name << ": " << val;
        
        if (prop == cv::CAP_PROP_FOURCC && val > 0) {
            int fourccVal = static_cast<int>(val);
            char fourccStr[5] = {
                static_cast<char>(fourccVal & 0xFF),
                static_cast<char>((fourccVal >> 8) & 0xFF),
                static_cast<char>((fourccVal >> 16) & 0xFF),
                static_cast<char>((fourccVal >> 24) & 0xFF),
                '\0'
            };
            std::cout << " (" << fourccStr << ")";
        }
        std::cout << std::endl;
    };
    
    printProp(cv::CAP_PROP_FRAME_WIDTH, "Width");
    printProp(cv::CAP_PROP_FRAME_HEIGHT, "Height");
    printProp(cv::CAP_PROP_FPS, "FPS");
    printProp(cv::CAP_PROP_FOURCC, "FourCC");
    printProp(cv::CAP_PROP_BUFFERSIZE, "Buffer Size");
    printProp(cv::CAP_PROP_EXPOSURE, "Exposure");
    printProp(cv::CAP_PROP_GAIN, "Gain");
    
    std::cout << "==================" << std::endl;
}

bool CameraSource::setBestMode(int targetWidth, int targetHeight, double targetFps) {
    if (!capture.isOpened()) return false;
    
    std::cout << "Setting mode: " << targetWidth << "x" << targetHeight 
              << " @ " << targetFps << " FPS" << std::endl;
    
    capture.set(cv::CAP_PROP_FRAME_WIDTH, targetWidth);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, targetHeight);
    capture.set(cv::CAP_PROP_FPS, targetFps);
    
    // Update stored values
    width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    fps = capture.get(cv::CAP_PROP_FPS);
    
    std::cout << "Actual mode: " << width << "x" << height << " @ " << fps << " FPS" << std::endl;
    
    return true;
}