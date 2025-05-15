#include <iostream>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>

#include <sys/stat.h>
#include <sstream>
#include <iomanip>

#ifdef _WIN32
#include <direct.h>
#endif

#include "ImageSourceFactory.h"
#include "aruco_pose_pipeline.h"
#include "Logger.h"  // Include Logger header

// Global function to create logs directory if it doesn't exist
void ensureLogDirectoryExists() {
    std::string dirPath = "logs";
    struct stat info;
    
    // Check if directory exists and create it if it doesn't
    if (stat(dirPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        #ifdef _WIN32
        _mkdir(dirPath.c_str());
        #else
        mkdir(dirPath.c_str(), 0755);
        #endif
    }
}


// Display the detection results
void displayResults(const ArucoPoseResult& result, bool transformed = false) {
    // Original displayResults code remains the same
    std::cout << "\n========== ArUco Detection Results ==========" << std::endl;
    std::cout << "Detection valid: " << (result.detectionValid ? "YES" : "NO") << std::endl;
    
    if (result.detectionValid) {
        std::cout << "Marker ID: " << result.markerId << std::endl;
        std::cout << "Reprojection error: " << result.reprojectionError << std::endl;
        
        std::cout << "Position (x,y,z): " 
                  << result.position.x() << ", " 
                  << result.position.y() << ", " 
                  << result.position.z() << std::endl;
        
        std::cout << "Orientation (quaternion w,x,y,z): " 
                  << result.orientation.w() << ", " 
                  << result.orientation.x() << ", " 
                  << result.orientation.y() << ", " 
                  << result.orientation.z() << std::endl;
    }
    else {
        // Log invalid detection
        UAV::logger().Write("ARUC", "TimeUS,Valid", "QB",
                          UAV::logger().getMicroseconds(),
                          0); // Not valid
    }
    
    std::cout << "Processing time: " << result.processingTimeMs << " ms" << std::endl;
    std::cout << "FPS: " << result.fps << std::endl;
    
    if (transformed) {
        std::cout << "(Values shown in drone reference frame)" << std::endl;
    }
    
    std::cout << "===========================================" << std::endl;
}

// Test ArUco detection with a specific image source
void testArucoDetection(ImageSourcePtr source, const std::string& windowName, int maxFrames = 1000) {
    if (!source || !source->initialize()) {
        std::cerr << "Failed to initialize image source" << std::endl;
        return;
    }
    
    // Log source initialization
    UAV::logger().Write("SRCE", "TimeUS,Name,Width,Height,FPS", "QZIIf",
                       UAV::logger().getMicroseconds(),
                       windowName.c_str(),
                       source->getWidth(),
                       source->getHeight(),
                       source->getFPS());
    
    // Create and configure the ArUco pipeline
    ArucoPosePipeline pipeline;
    
    // Configure settings
    ArucoPoseSettings settings;
    settings.dictionaryId = cv::aruco::DICT_5X5_1000;
    settings.markerSizeMeters = 0.5;  // 5cm marker
    settings.useCornerRefinement = true;
    settings.cornerRefinementMaxIterations = 30;
    settings.cornerRefinementMinAccuracy = 0.01;
    settings.maxReprojectionError = 2.0;
    settings.debugVisualization = true;
    
    // Update pipeline with settings
    pipeline.updateSettings(settings);
    
    // Log ArUco configuration
    UAV::logger().Write("ACFG", "TimeUS,DictID,MarkerSize,UseCornerRef,MaxError", "QIfBf",
                       UAV::logger().getMicroseconds(),
                       settings.dictionaryId,
                       settings.markerSizeMeters,
                       settings.useCornerRefinement ? 1 : 0,
                       settings.maxReprojectionError);
    
    // Camera calibration data from gazebo for 640x480
    CameraCalibration calibration;
    calibration.cameraMatrix = (cv::Mat_<double>(3, 3) << 
        205.46962738037109, 0.0, 320.0,
        0.0, 205.46965599060059, 240.0,
        0.0, 0.0, 1.0);
    calibration.distCoeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
    calibration.cameraMatrix.convertTo(calibration.cameraMatrix, CV_64F);
    calibration.distCoeffs.convertTo(calibration.distCoeffs, CV_64F);
    pipeline.setCalibration(calibration);
    
    // Create drone reference frame transformation
    Eigen::Matrix4d droneTransform;
    droneTransform << 0, -1, 0, 0,
                      1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    
    // Main processing loop
    cv::Mat frame;
    int frameCount = 0;
    int validDetections = 0;
    
    // For FPS calculation
    auto startTime = std::chrono::steady_clock::now();
    int fpsCounter = 0;
    double actualFps = 0;
    
    std::cout << "Processing frames from " << windowName << "..." << std::endl;
    
    // Log start of processing
    UAV::logger().Write("STRT", "TimeUS,Source", "QZ",
                       UAV::logger().getMicroseconds(),
                       windowName.c_str());
    
    while (frameCount < maxFrames) {
        if (source->getNextFrame(frame)) {
            if (!frame.empty()) {
                // Prepare input frame
                ArucoInputFrame inputFrame;
                inputFrame.image = frame.clone();
                inputFrame.sequenceId = frameCount;
                inputFrame.timestamp = std::chrono::steady_clock::now();
                
                // Process the frame
                ArucoPoseResult result = pipeline.process(inputFrame);
                result.applyTransform(droneTransform);
                
                // Log frame processing on every frame
                UAV::logger().Write("FRAM", "TimeUS,SeqID,ProcTimeMs,FPS", "QIfd",
                                   UAV::logger().getMicroseconds(),
                                   frameCount,
                                   result.processingTimeMs,
                                   result.fps);
                                   if (result.detectionValid) {
                UAV::logger().Write("POSE", "TimeUS,PosX,PosY,PosZ,QuatW,QuatX,QuatY,QuatZ", "Qfffffff",
                                    UAV::logger().getMicroseconds(),
                                    static_cast<float>(result.position.x()),
                                    static_cast<float>(result.position.y()),
                                    static_cast<float>(result.position.z()),
                                    static_cast<float>(result.orientation.w()),
                                    static_cast<float>(result.orientation.x()),
                                    static_cast<float>(result.orientation.y()),
                                    static_cast<float>(result.orientation.z()));
                }
                // // Display results (only every 30 frames to avoid console spam)
                // if (frameCount % 30 == 0) {
                //     // displayResults(result);
                    
                //     // If detection is valid, also show transformed result
                //     if (result.detectionValid) {
                //         // Create a copy of the result
                //         ArucoPoseResult transformedResult = result;
                        
                //         // Apply drone transformation
                //         transformedResult.applyTransform(droneTransform);
                        
                //         // Display transformed result
                //         std::cout << "\nTransformed to drone reference frame:" << std::endl;
                //         displayResults(transformedResult, true);
                //     }
                // }
                
                // Count valid detections
                if (result.detectionValid) {
                    validDetections++;
                }
                
                // Display the frame with debug visualization
                if (!result.debugImage.empty()) {
                    cv::imshow(windowName, result.debugImage);
                } else {
                    cv::imshow(windowName, frame);
                }
                
                // Update frame counter
                frameCount++;
                fpsCounter++;
                
                // Calculate FPS every second
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - startTime).count();
                
                if (elapsed >= 1000) {
                    actualFps = fpsCounter * 1000.0 / elapsed;
                    
                    // Log FPS stats
                    UAV::logger().Write("AFPS", "TimeUS,FPS,Target", "Qff",
                                       UAV::logger().getMicroseconds(),
                                       actualFps,
                                       source->getFPS());
                    
                    std::cout << "Application FPS: " << actualFps 
                              << " (Target: " << source->getFPS() << ")" << std::endl;
                    
                    fpsCounter = 0;
                    startTime = now;
                }
            }
        }
        
        // Check for key press (27 = ESC key)
        if (cv::waitKey(1) == 27) {
            // Log user interrupt
            UAV::logger().Write("INTR", "TimeUS,Reason", "QZ",
                               UAV::logger().getMicroseconds(),
                               "User pressed ESC");
            break;
        }
    }
    
    // Print final statistics
    std::cout << "\nProcessed " << frameCount << " frames" << std::endl;
    std::cout << "Valid detections: " << validDetections 
              << " (" << (frameCount > 0 ? (validDetections * 100.0 / frameCount) : 0)
              << "%)" << std::endl;
    
    // Log final stats
    UAV::logger().Write("STAT", "TimeUS,TotalFrames,ValidDetections,Percentage", "QIIf",
                       UAV::logger().getMicroseconds(),
                       frameCount,
                       validDetections,
                       (frameCount > 0 ? (validDetections * 100.0 / frameCount) : 0));
    
    // Release resources
    cv::destroyWindow(windowName);
    source->release();
}

int main(int argc, char** argv) {
    // Ensure logs directory exists
    ensureLogDirectoryExists();
    
    // Generate log filename
    
    // Initialize logger
    if (!UAV::logger().initialize("logs", "aruco_test")) {
        std::cerr << "Error: Failed to initialize logger" << std::endl;
        return 1;
    }
    
    // Log application start
    UAV::logger().Write("BOOT", "TimeUS,Version", "QZ",
                       UAV::logger().getMicroseconds(),
                       "ArUco Pose Test 1.0");
    
    // Parse command line arguments
    bool useGazebo = true;  // Default to Gazebo for testing
    std::string gazeboTopic = "/world/map/model/iris/link/camera_link/sensor/camera/image";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--camera" || arg == "-c") {
            useGazebo = false;
            // Log command-line option
            UAV::logger().Write("OPTS", "TimeUS,Option,Value", "QZZ",
                               UAV::logger().getMicroseconds(),
                               "source",
                               "camera");
        } else if (arg == "--topic" || arg == "-t") {
            if (i + 1 < argc) {
                gazeboTopic = argv[i + 1];
                // Log command-line option
                UAV::logger().Write("OPTS", "TimeUS,Option,Value", "QZZ",
                                   UAV::logger().getMicroseconds(),
                                   "topic",
                                   gazeboTopic.c_str());
                i++;
            }
        }
    }
    
    // Create the appropriate image source
    ImageSourcePtr source;
    std::string windowName;
    
    if (useGazebo) {
        std::cout << "Using Gazebo source with topic: " << gazeboTopic << std::endl;
        source = ImageSourceFactory::createGazeboSource(gazeboTopic);
        windowName = "ArUco Detection (Gazebo)";
    } else {
        std::cout << "Using physical camera" << std::endl;
        source = ImageSourceFactory::createCameraSource();
        windowName = "ArUco Detection (Camera)";
    }
    
    // Test ArUco detection with the selected source
    testArucoDetection(source, windowName);
    
    // Log application exit
    UAV::logger().Write("EXIT", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Normal exit");
    
    // Close the logger
    UAV::logger().close();
    
    std::cout << "Logs saved to: " << UAV::logger().getLogDirectory() << std::endl;
    
    return 0;
}