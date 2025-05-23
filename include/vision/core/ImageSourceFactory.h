// include/ImageSourceFactory.h
#ifndef IMAGE_SOURCE_FACTORY_H
#define IMAGE_SOURCE_FACTORY_H

#include "ImageSourceInterface.h"
#include "CameraSource.h"
#ifdef WITH_GAZEBO
#include "GazeboSource.h"
#endif
#include "parameters.h"
#include <memory>
#include <string>


/**
 * @class ImageSourceFactory
 * @brief Factory for creating different types of image sources
 */
class ImageSourceFactory {
public:
    /**
     * Enumeration of available image source types
     */
    enum class SourceType {
        CAMERA,        ///< Physical camera
        GAZEBO         ///< Gazebo simulation camera
    };
    
    /**
     * Create an image source of the specified type
     * 
     * @param type Type of image source to create
     * @param width Desired image width
     * @param height Desired image height
     * @param fps Desired frames per second
     * @return Shared pointer to the created image source
     */
    static ImageSourcePtr createSource(
        SourceType type,
        int width = 640,
        int height = 480,
        double fps = 30.0) {
        
        switch (type) {
            case SourceType::CAMERA:
                return std::make_shared<CameraSource>(0, width, height, fps);
                
            case SourceType::GAZEBO:
                #ifdef WITH_GAZEBO
                return std::make_shared<GazeboSource>(
                    "/world/default/model/iris/link/camera_link/sensor/camera/image",
                    width, height, fps);
                #else
                std::cerr << "Error: Gazebo support not compiled in this build" << std::endl;
                return nullptr;
                #endif
                
            default:
                return nullptr;
        }
    }
    
    /**
     * Create a camera source with specific parameters
     * 
     * @param cameraId Camera ID or index
     * @param width Desired width
     * @param height Desired height
     * @param fps Desired frames per second
     * @param apiPreference OpenCV camera API preference
     * @return Shared pointer to the created camera source
     */
    static ImageSourcePtr createCameraSource(
        int cameraId = 0,
        int width = 1280,
        int height = 720,
        double fps = 50.0,
        int apiPreference = cv::CAP_V4L2) {
        
        return std::make_shared<CameraSource>(cameraId, width, height, fps, apiPreference);
    }
    
    
    // Add this to ImageSourceFactory.h for optimized camera creation

    /**
     * Create an optimized camera source for maximum performance
     * 
     * @param cameraId Camera ID or index
     * @param width Desired width
     * @param height Desired height
     * @param fps Desired frames per second
     * @return Shared pointer to the optimized camera source
     */
    static ImageSourcePtr createOptimizedCameraSource(
        int cameraId = 0,
        int width = 640,
        int height = 480,
        double fps = 100.0) {  // Default to high FPS
        
        auto camera = std::make_shared<CameraSource>(
            cameraId, width, height, fps, 
            cv::CAP_V4L2,  // V4L2 is usually fastest on Linux
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G')  // MJPEG for speed
        );
        
        // Configure for maximum performance
        camera->setShouldCopyFrame(false);  // Zero-copy when possible
        camera->setPreprocessType(FramePreprocessType::RAW);  // No preprocessing
        
        return camera;
    }

    /**
     * Create camera source based on use case
     */
    static ImageSourcePtr createCameraForUseCase(
        const std::string& useCase,
        int cameraId = 0) {
        
        if (useCase == "aruco_tracking") {
            // Optimized for ArUco marker tracking
            auto camera = createOptimizedCameraSource(cameraId, 640, 480, 100.0);
            
            // ArUco works well with grayscale
            std::dynamic_pointer_cast<CameraSource>(camera)->setPreprocessType(
                FramePreprocessType::GRAYSCALE);
            
            return camera;
        }
        else if (useCase == "high_resolution") {
            // High resolution but lower FPS
            return createOptimizedCameraSource(cameraId, 1280, 720, 30.0);
        }
        else if (useCase == "low_latency") {
            // Minimum latency configuration
            auto camera = createOptimizedCameraSource(cameraId, 320, 240, 120.0);
            return camera;
        }
        else {
            // Default balanced configuration
            return createOptimizedCameraSource(cameraId, 640, 480, 60.0);
        }
    }

    /**
     * Create a Gazebo source with specific parameters
     * 
     * @param topicName Gazebo camera topic to subscribe to
     * @param width Expected frame width (0 for auto-detect)
     * @param height Expected frame height (0 for auto-detect)
     * @param fps Desired frames per second
     * @return Shared pointer to the created Gazebo source
     */
    static ImageSourcePtr createGazeboSource(
        const std::string& topicName = UAV_Parameters::GZ_CAM_TOPIC,
        int width = 0,
        int height = 0,
        double fps = 50.0) {
        
        #ifdef WITH_GAZEBO
        return std::make_shared<GazeboSource>(topicName, width, height, fps);
        #else
        std::cerr << "Error: Gazebo support not compiled in this build" << std::endl;
        return nullptr;
        #endif
    }
};

#endif // IMAGE_SOURCE_FACTORY_H
