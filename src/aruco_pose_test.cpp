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
#include "aruco_ekf_estimator.h"
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

// Display the EKF state estimation results
void displayEKFResults(const EKFStateResult& result) {
    std::cout << "\n========== EKF State Estimation Results ==========" << std::endl;
    std::cout << "Valid: " << (result.valid ? "YES" : "NO") << std::endl;
    
    if (result.valid) {
        std::cout << "Position (x,y,z): " 
                  << result.position.x() << ", " 
                  << result.position.y() << ", " 
                  << result.position.z() 
                  << " (±" << result.positionStdDev.x() << ", ±" 
                  << result.positionStdDev.y() << ", ±" 
                  << result.positionStdDev.z() << ")" << std::endl;
        
        std::cout << "Velocity (vx,vy,vz): " 
                  << result.velocity.x() << ", " 
                  << result.velocity.y() << ", " 
                  << result.velocity.z() 
                  << " (±" << result.velocityStdDev.x() << ", ±" 
                  << result.velocityStdDev.y() << ", ±" 
                  << result.velocityStdDev.z() << ")" << std::endl;
        
        std::cout << "Acceleration (ax,ay,az): " 
                  << result.acceleration.x() << ", " 
                  << result.acceleration.y() << ", " 
                  << result.acceleration.z() << std::endl;
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

// Test ArUco detection with EKF state estimation
void testArucoEKF(ImageSourcePtr source, const std::string& windowName, int maxFrames = 1000) {
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
    settings.markerSizeMeters = 0.5;  // 50cm marker
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
    
    // Create and configure EKF estimator
    EKFEstimatorConfig ekfConfig;
    ekfConfig.predictionFrequencyHz = 100.0;  // 100 Hz prediction rate
    ekfConfig.positionProcessNoise = 0.01;    // Position process noise
    ekfConfig.velocityProcessNoise = 0.1;     // Velocity process noise
    ekfConfig.positionMeasurementNoise = 0.01; // Position measurement noise
    
    ArucoEKFEstimator ekf(ekfConfig);
    
    // Log EKF configuration
    UAV::logger().Write("EKFC", "TimeUS,PredFreq,PosNoise,VelNoise,MeasNoise", "Qffff",
                       UAV::logger().getMicroseconds(),
                       static_cast<float>(ekfConfig.predictionFrequencyHz),
                       static_cast<float>(ekfConfig.positionProcessNoise),
                       static_cast<float>(ekfConfig.velocityProcessNoise),
                       static_cast<float>(ekfConfig.positionMeasurementNoise));
    
    // Main processing loop
    cv::Mat frame;
    int frameCount = 0;
    int validDetections = 0;
    int ekfUpdates = 0;
    int ekfPredictions = 0;
    
    // For FPS calculation
    auto startTime = std::chrono::steady_clock::now();
    int fpsCounter = 0;
    double actualFps = 0;
    
    // For EKF prediction timing
    auto lastEkfPredictionTime = startTime;
    
    std::cout << "Processing frames with EKF estimation from " << windowName << "..." << std::endl;
    
    // Log start of processing
    UAV::logger().Write("STRT", "TimeUS,Source,Mode", "QZZ",
                       UAV::logger().getMicroseconds(),
                       windowName.c_str(),
                       "EKF");
    
    while (frameCount < maxFrames) {
        auto currentTime = std::chrono::steady_clock::now();
        
        // Run EKF prediction at a higher frequency (100 Hz)
        if (ekf.isInitialized()) {
            auto timeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                currentTime - lastEkfPredictionTime).count();
                
            // Run prediction step at predictionFrequencyHz
            if (timeElapsed >= (1000.0 / ekfConfig.predictionFrequencyHz)) {
                if (ekf.predict(currentTime)) {
                    ekfPredictions++;
                    
                    // Get and log state estimate
                    EKFStateResult ekfState = ekf.getStateEstimate();
                    
                    // Log EKF prediction state
                    UAV::logger().Write("EKFP", "TimeUS,PosX,PosY,PosZ,VelX,VelY,VelZ", "Qffffff",
                                      UAV::logger().getMicroseconds(),
                                      static_cast<float>(ekfState.position.x()),
                                      static_cast<float>(ekfState.position.y()),
                                      static_cast<float>(ekfState.position.z()),
                                      static_cast<float>(ekfState.velocity.x()),
                                      static_cast<float>(ekfState.velocity.y()),
                                      static_cast<float>(ekfState.velocity.z()));
                }
                
                lastEkfPredictionTime = currentTime;
            }
        }
        
        // Get new frame and process with ArUco detector
        if (source->getNextFrame(frame)) {
            if (!frame.empty()) {
                // Prepare input frame
                ArucoInputFrame inputFrame;
                inputFrame.image = frame.clone();
                inputFrame.sequenceId = frameCount;
                inputFrame.timestamp = currentTime;
                
                // Process the frame with ArUco detection
                ArucoPoseResult result = pipeline.process(inputFrame);
                
                // Apply drone reference frame transformation
                result.applyTransform(droneTransform);
                
                // Log frame processing
                UAV::logger().Write("FRAM", "TimeUS,SeqID,ProcTimeMs,FPS", "QIfd",
                                   UAV::logger().getMicroseconds(),
                                   frameCount,
                                   result.processingTimeMs,
                                   result.fps);
                
                // Log ArUco pose if detection is valid
                if (result.detectionValid) {
                    validDetections++;
                    
                    UAV::logger().Write("APOS", "TimeUS,PosX,PosY,PosZ,QuatW,QuatX,QuatY,QuatZ", "Qfffffff",
                                      UAV::logger().getMicroseconds(),
                                      static_cast<float>(result.position.x()),
                                      static_cast<float>(result.position.y()),
                                      static_cast<float>(result.position.z()),
                                      static_cast<float>(result.orientation.w()),
                                      static_cast<float>(result.orientation.x()),
                                      static_cast<float>(result.orientation.y()),
                                      static_cast<float>(result.orientation.z()));
                    
                    // Update EKF with ArUco measurement
                    if (ekf.update(result, currentTime)) {
                        ekfUpdates++;
                        
                        // Log EKF update
                        UAV::logger().Write("EKFU", "TimeUS,SeqID", "QI",
                                          UAV::logger().getMicroseconds(),
                                          frameCount);
                        
                        // Get and log updated state estimate
                        EKFStateResult ekfState = ekf.getStateEstimate();
                        
                        // Log EKF state after update
                        UAV::logger().Write("EKFS", "TimeUS,PosX,PosY,PosZ,VelX,VelY,VelZ,StdX,StdY,StdZ", "Qfffffffff",
                                          UAV::logger().getMicroseconds(),
                                          static_cast<float>(ekfState.position.x()),
                                          static_cast<float>(ekfState.position.y()),
                                          static_cast<float>(ekfState.position.z()),
                                          static_cast<float>(ekfState.velocity.x()),
                                          static_cast<float>(ekfState.velocity.y()),
                                          static_cast<float>(ekfState.velocity.z()),
                                          static_cast<float>(ekfState.positionStdDev.x()),
                                          static_cast<float>(ekfState.positionStdDev.y()),
                                          static_cast<float>(ekfState.positionStdDev.z()));
                        
                        // Display state estimate every 30 frames
                        if (frameCount % 30 == 0) {
                            displayEKFResults(ekfState);
                        }
                    }
                }
                
                // Create visualization frame
                cv::Mat displayFrame;
                if (!result.debugImage.empty()) {
                    displayFrame = result.debugImage.clone();
                } else {
                    displayFrame = frame.clone();
                }
                
                // Add EKF state information to display
                if (ekf.isInitialized()) {
                    EKFStateResult ekfState = ekf.getStateEstimate();
                    
                    // Position and velocity
                    cv::putText(displayFrame, 
                              "EKF Pos: " + std::to_string(ekfState.position.x()) + ", " 
                                          + std::to_string(ekfState.position.y()) + ", " 
                                          + std::to_string(ekfState.position.z()),
                              cv::Point(10, 150), 
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
                    
                    cv::putText(displayFrame, 
                              "EKF Vel: " + std::to_string(ekfState.velocity.x()) + ", " 
                                          + std::to_string(ekfState.velocity.y()) + ", " 
                                          + std::to_string(ekfState.velocity.z()),
                              cv::Point(10, 180), 
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
                    
                    // Status info
                    cv::putText(displayFrame, 
                              "EKF Updates: " + std::to_string(ekfUpdates) + " / Predictions: " 
                                              + std::to_string(ekfPredictions),
                              cv::Point(10, 210), 
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
                }
                
                // Display the frame
                cv::imshow(windowName, displayFrame);
                
                // Update frame counter
                frameCount++;
                fpsCounter++;
                
                // Calculate FPS every second
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    currentTime - startTime).count();
                
                if (elapsed >= 1000) {
                    actualFps = fpsCounter * 1000.0 / elapsed;
                    
                    // Log FPS stats
                    UAV::logger().Write("AFPS", "TimeUS,FPS,Target,EKFPredRate", "Qfff",
                                       UAV::logger().getMicroseconds(),
                                       actualFps,
                                       source->getFPS(),
                                       static_cast<float>(ekfPredictions) * 1000.0f / elapsed);
                    
                    std::cout << "Application FPS: " << actualFps 
                              << " (Target: " << source->getFPS() << ")" << std::endl;
                    std::cout << "EKF Prediction Rate: " << (ekfPredictions * 1000.0 / elapsed)
                              << " Hz (Target: " << ekfConfig.predictionFrequencyHz << " Hz)" << std::endl;
                    
                    fpsCounter = 0;
                    ekfPredictions = 0;
                    startTime = currentTime;
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
    std::cout << "EKF updates: " << ekfUpdates 
              << " (" << (frameCount > 0 ? (ekfUpdates * 100.0 / frameCount) : 0)
              << "%)" << std::endl;
    
    // Log final stats
    UAV::logger().Write("STAT", "TimeUS,TotalFrames,ValidDetections,EKFUpdates", "QIII",
                       UAV::logger().getMicroseconds(),
                       frameCount,
                       validDetections,
                       ekfUpdates);
    
    // Release resources
    cv::destroyWindow(windowName);
    source->release();
}

int main(int argc, char** argv) {
    // Ensure logs directory exists
    ensureLogDirectoryExists();
    
    // Generate log filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << "aruco_test_" << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    std::string logPrefix = ss.str();
    
    // Initialize logger
    if (!UAV::logger().initialize("logs", logPrefix.c_str())) {
        std::cerr << "Error: Failed to initialize logger" << std::endl;
        return 1;
    }
    
    // Log application start
    UAV::logger().Write("BOOT", "TimeUS,Version", "QZ",
                       UAV::logger().getMicroseconds(),
                       "ArUco Pose Test with EKF 1.0");
    
    // Parse command line arguments
    bool useGazebo = true;    // Default to Gazebo for testing
    bool useEKF = true;      // Whether to use EKF estimation
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
        } else if (arg == "--ekf" || arg == "-e") {
            useEKF = true;
            // Log command-line option
            UAV::logger().Write("OPTS", "TimeUS,Option,Value", "QZZ",
                               UAV::logger().getMicroseconds(),
                               "mode",
                               "ekf");
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "ArUco Pose Test with EKF\n\n"
                      << "Options:\n"
                      << "  -c, --camera       Use physical camera instead of Gazebo\n"
                      << "  -t, --topic TOPIC  Set Gazebo camera topic\n"
                      << "  -e, --ekf          Enable EKF state estimation\n"
                      << "  -h, --help         Show this help message\n"
                      << std::endl;
            return 0;
        }
    }
    
    // Create the appropriate image source
    ImageSourcePtr source;
    std::string windowName;
    
    if (useGazebo) {
        std::cout << "Using Gazebo source with topic: " << gazeboTopic << std::endl;
        source = ImageSourceFactory::createGazeboSource(gazeboTopic);
        windowName = useEKF ? "ArUco Detection with EKF (Gazebo)" : "ArUco Detection (Gazebo)";
    } else {
        std::cout << "Using physical camera" << std::endl;
        source = ImageSourceFactory::createCameraSource();
        windowName = useEKF ? "ArUco Detection with EKF (Camera)" : "ArUco Detection (Camera)";
    }
    
    // Run the appropriate test
    if (useEKF) {
        testArucoEKF(source, windowName);
    } else {
        testArucoDetection(source, windowName);
    }
    
    // Log application exit
    UAV::logger().Write("EXIT", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Normal exit");
    
    // Close the logger
    UAV::logger().close();
    
    std::cout << "Logs saved to: " << UAV::logger().getLogDirectory() << "/" << logPrefix << std::endl;
    
    return 0;
}