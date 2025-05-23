// include/vision/core/CameraSource.h
#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include "ImageSourceInterface.h"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>

// Frame preprocessing type (from old FrameProvider)
enum class FramePreprocessType {
    RAW,
    GRAYSCALE,
    HSV,
    THRESHOLD
};

/**
 * @class CameraSource
 * @brief High-performance camera source with minimal overhead
 * 
 * Optimized for low-latency frame acquisition based on the original FrameProvider design
 */
class CameraSource : public ImageSourceInterface {
public:
    /**
     * Constructor
     * @param cameraId Camera ID or index
     * @param width Desired width
     * @param height Desired height
     * @param fps Desired FPS
     * @param apiPreference OpenCV camera API preference
     * @param fourcc FourCC code
     */
    CameraSource(int cameraId = 0, 
                int width = 640, 
                int height = 480, 
                double fps = 30.0,
                int apiPreference = cv::CAP_V4L2,
                int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    /**
     * Destructor
     */
    ~CameraSource() override;
    
    // ImageSourceInterface implementation
    bool initialize() override;
    bool isInitialized() const override;
    bool getNextFrame(cv::Mat& frame) override;
    int getWidth() const override;
    int getHeight() const override;
    double getFPS() const override;
    void release() override;
    
    // Performance optimization settings
    /**
     * Set preprocessing type (for reduced processing overhead)
     */
    void setPreprocessType(FramePreprocessType type);
    
    /**
     * Set whether to copy frames (false for zero-copy when possible)
     */
    void setShouldCopyFrame(bool copy);
    
    /**
     * Set rotation in degrees (0, 90, 180, 270)
     */
    void setRotation(int degrees);
    
    // Camera-specific methods
    void printCameraModes();
    bool setBestMode(int targetWidth, int targetHeight, double targetFps);
    bool setExposure(int value);
    bool setAutoExposure(bool enable);
    
private:
    cv::VideoCapture capture;    
    
    // Configuration
    int cameraId;
    int width;
    int height;
    double fps;
    int apiPreference;
    int fourcc;
    
    // Optimization settings
    FramePreprocessType preprocessType;
    bool shouldCopyFrame;
    int rotation;
    
    // State
    std::atomic<bool> initialized;
    
    // Performance optimization: no mutex needed for VideoCapture::read()
    // as it's generally thread-safe for single reader
    
    // Cached frame for zero-copy when possible
    cv::Mat cachedFrame;
    
    // Internal helper
    void preprocessFrame(cv::Mat& frame);
    void applyRotation(cv::Mat& frame);
};

#endif // CAMERA_SOURCE_H 