#include <iostream>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

#include "ImageSourceFactory.h"
#include "aruco_pose_pipeline.h"


// Display the detection results
void displayResults(const ArucoPoseResult& result, bool transformed = false) {
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
    
    /*Camera calibration data from gazebo for 640x480:
    intrinsics {
  k: 205.46962738037109
  k: 0
  k: 320
  k: 0
  k: 205.46965599060059
  k: 240
  k: 0
  k: 0
  k: 1
}
*/
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
    /*Custom drone transform
    0, -1, 0, 0//
    1, 0, 0, 0//
    0, 0, 1, 0//
    0, 0, 0, 1*/
    Eigen::Matrix4d droneTransform;
    droneTransform << 0, -1, 0, 0,
                      1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    // Set the drone transform in the pipeline
    
    // Main processing loop
    cv::Mat frame;
    int frameCount = 0;
    int validDetections = 0;
    
    // For FPS calculation
    auto startTime = std::chrono::steady_clock::now();
    int fpsCounter = 0;
    double actualFps = 0;
    
    std::cout << "Processing frames from " << windowName << "..." << std::endl;
    
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
                
                // Display results (only every 30 frames to avoid console spam)
                if (frameCount % 30 == 0) {
                    displayResults(result);
                    
                    // If detection is valid, also show transformed result
                    if (result.detectionValid) {
                        // Create a copy of the result
                        ArucoPoseResult transformedResult = result;
                        
                        // Apply drone transformation
                        transformedResult.applyTransform(droneTransform);
                        
                        // Display transformed result
                        std::cout << "\nTransformed to drone reference frame:" << std::endl;
                        displayResults(transformedResult, true);
                    }
                }
                
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
                    std::cout << "Application FPS: " << actualFps 
                              << " (Target: " << source->getFPS() << ")" << std::endl;
                    
                    fpsCounter = 0;
                    startTime = now;
                }
            }
        }
        
        // Check for key press (27 = ESC key)
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    
    // Print final statistics
    std::cout << "\nProcessed " << frameCount << " frames" << std::endl;
    std::cout << "Valid detections: " << validDetections 
              << " (" << (frameCount > 0 ? (validDetections * 100.0 / frameCount) : 0)
              << "%)" << std::endl;
    
    // Release resources
    cv::destroyWindow(windowName);
    source->release();
}

int main(int argc, char** argv) {
    // Parse command line arguments
    bool useGazebo = true;  // Default to Gazebo for testing
    std::string gazeboTopic = "/world/map/model/iris/link/camera_link/sensor/camera/image";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--camera" || arg == "-c") {
            useGazebo = false;
        } else if (arg == "--topic" || arg == "-t") {
            if (i + 1 < argc) {
                gazeboTopic = argv[i + 1];
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
    
    return 0;
}