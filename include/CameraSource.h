// include/CameraSource.h
#ifndef CAMERA_SOURCE_H
#define CAMERA_SOURCE_H

#include "ImageSourceInterface.h"
#include <opencv2/opencv.hpp>

/**
 * @class CameraSource
 * @brief Provides frames from a physical camera
 * 
 * Uses OpenCV's VideoCapture to access physical cameras
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
    
private:
    cv::VideoCapture capture;  // OpenCV camera capture object
    int cameraId;              // Camera ID/index
    int width;                 // Desired width
    int height;                // Desired height
    double fps;                // Desired FPS
    int apiPreference;         // OpenCV camera API preference
    int fourcc;               // FourCC code for video encoding
    bool initialized;          // Initialization flag
};

#endif // CAMERA_SOURCE_H
