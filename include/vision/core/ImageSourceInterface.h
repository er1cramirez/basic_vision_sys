// include/ImageSourceInterface.h
#ifndef IMAGE_SOURCE_INTERFACE_H
#define IMAGE_SOURCE_INTERFACE_H

#include <opencv2/opencv.hpp>
#include <memory>

/**
 * @class ImageSourceInterface
 * @brief Abstract interface for different image sources
 * 
 * This interface provides a unified API for retrieving frames from
 * different image sources (camera, Gazebo, etc.)
 */
class ImageSourceInterface {
public:
    /**
     * Virtual destructor
     */
    virtual ~ImageSourceInterface() = default;

    /**
     * Initialize the image source with given parameters
     * @return true if initialization successful, false otherwise
     */
    virtual bool initialize() = 0;

    /**
     * Check if the source is properly initialized and ready to provide frames
     * @return true if ready, false otherwise
     */
    virtual bool isInitialized() const = 0;
    
    /**
     * Get the next frame from the source
     * @param frame OpenCV Mat where the frame will be stored
     * @return true if frame was retrieved successfully, false otherwise
     */
    virtual bool getNextFrame(cv::Mat& frame) = 0;
    
    /**
     * Get the width of the frames provided by this source
     * @return frame width in pixels
     */
    virtual int getWidth() const = 0;
    
    /**
     * Get the height of the frames provided by this source
     * @return frame height in pixels
     */
    virtual int getHeight() const = 0;
    
    /**
     * Get the FPS (frames per second) of this source
     * @return frames per second (may be estimated)
     */
    virtual double getFPS() const = 0;
    
    /**
     * Release all resources used by this source
     */
    virtual void release() = 0;
};

/**
 * Type definition for a shared pointer to an ImageSourceInterface
 */
using ImageSourcePtr = std::shared_ptr<ImageSourceInterface>;

#endif // IMAGE_SOURCE_INTERFACE_H
