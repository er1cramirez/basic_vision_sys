// include/CameraSource.h
#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include "ImageSourceInterface.h"
#include <opencv2/opencv.hpp>
#include <mutex>

/**
 * @class CameraSource
 * @brief Provides frames from a physical camera
 * 
 * Uses OpenCV's VideoCapture to access physical cameras with improved
 * error handling and thread safety
 */
class CameraSource : public ImageSourceInterface {
public:
    /**
     * Constructor
     * @param cameraId Camera ID or index (usually 0 for the default camera)
     * @param width Desired width of the camera frames
     * @param height Desired height of the camera frames
     * @param fps Desired FPS (frames per second)
     * @param apiPreference OpenCV camera API preference
     * @param fourcc FourCC code for video encoding (default MJPG)
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
    
    // Camera-specific methods
    /**
     * Display all supported camera modes (resolution and FPS)
     */
    void printCameraModes();
    
    /**
     * Attempt to set the best available mode
     * @param targetWidth Desired width
     * @param targetHeight Desired height  
     * @param targetFps Desired FPS
     * @return true if settings were applied successfully
     */
    bool setBestMode(int targetWidth, int targetHeight, double targetFps);
    
    /**
     * Set camera exposure manually
     * @param value Exposure value
     * @return true if successful
     */
    bool setExposure(int value);
    
    /**
     * Enable or disable auto exposure
     * @param enable true to enable auto exposure
     * @return true if successful
     */
    bool setAutoExposure(bool enable);
    
private:
    cv::VideoCapture capture;    // OpenCV camera capture object
    mutable std::mutex captureMutex; // Mutex for thread-safe access
    
    int cameraId;                // Camera ID/index
    int width;                   // Current width
    int height;                  // Current height
    double fps;                  // Current FPS
    int apiPreference;           // OpenCV camera API preference
    int fourcc;                  // FourCC code for video encoding
    bool initialized;            // Initialization flag
    
    /**
     * Internal method to safely get camera property
     * @param prop OpenCV property ID
     * @return property value or -1 if unavailable
     */
    double safeGetProperty(int prop) const;
    
    /**
     * Internal method to safely set camera property
     * @param prop OpenCV property ID
     * @param value Value to set
     * @return true if successful
     */
    bool safeSetProperty(int prop, double value);
};

#endif // CAMERA_SOURCE_H