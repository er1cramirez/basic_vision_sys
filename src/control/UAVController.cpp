#include "UAVController.h"
#include "ImageSourceFactory.h"
#include "Logger.h"

#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <opencv2/highgui.hpp>

// Constructor
UAVController::UAVController()
    : running(false),
      stateMachine(stateData, controlData) {
          
    // Create log directory and prefix
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << "uav_control_" << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    logPrefix = ss.str();
    logDirectory = "logs";
}

// Destructor
UAVController::~UAVController() {
    // Stop threads if they're running
    if (running) {
        stop();
    }
}

// Initialize the controller
bool UAVController::initialize(const std::string& gazeboTopic, bool useCamera) {
    std::lock_guard<std::mutex> lock(controllerMutex);
    
    if (running) {
        std::cerr << "Controller already running, cannot initialize" << std::endl;
        return false;
    }
    
    // Initialize logger
    if (!UAV::logger().isInitialized()) {
        if (!UAV::logger().initialize(logDirectory, logPrefix)) {
            std::cerr << "Failed to initialize logger" << std::endl;
            return false;
        }
    }
    
    // Log initialization
    UAV::logger().Write("BOOT", "TimeUS,Version", "QZ",
                       UAV::logger().getMicroseconds(),
                       "UAV Controller 1.0");
    
    // Create image source
    if (useCamera) {
        imageSource = ImageSourceFactory::createCameraSource();
    } else {
        std::string topic = gazeboTopic.empty() ? 
            "/world/map/model/iris/link/camera_link/sensor/camera/image" : gazeboTopic;
        imageSource = ImageSourceFactory::createGazeboSource(topic);
    }
    
    // Initialize image source
    if (!imageSource || !imageSource->initialize()) {
        std::cerr << "Failed to initialize image source" << std::endl;
        return false;
    }
    
    // Setup ArUco pipeline
    setupArucoPipeline();
    
    // Setup EKF estimator
    setupEKFEstimator();
    
    // Initialize state machine
    if (!stateMachine.initialize()) {        
        std::cerr << "Failed to initialize state machine" << std::endl;
        return false;
    }    
    return true;
}

// Configure ArUco pipeline
void UAVController::setupArucoPipeline() {
    // Create and configure the ArUco pipeline
    ArucoPoseSettings settings;
    settings.dictionaryId = cv::aruco::DICT_5X5_1000;
    settings.markerSizeMeters = 0.5;  // 50cm marker
    settings.useCornerRefinement = true;
    settings.cornerRefinementMaxIterations = 30;
    settings.cornerRefinementMinAccuracy = 0.01;
    settings.maxReprojectionError = 2.0;
    settings.debugVisualization = true;
    
    // Update pipeline with settings
    arucoProcessor.updateSettings(settings);
    
    // Camera calibration data from gazebo for 640x480
    CameraCalibration calibration;
    calibration.cameraMatrix = (cv::Mat_<double>(3, 3) << 
        205.46962738037109, 0.0, 320.0,
        0.0, 205.46965599060059, 240.0,
        0.0, 0.0, 1.0);
    calibration.distCoeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
    calibration.cameraMatrix.convertTo(calibration.cameraMatrix, CV_64F);
    calibration.distCoeffs.convertTo(calibration.distCoeffs, CV_64F);
    arucoProcessor.setCalibration(calibration);
}

// Configure EKF estimator
void UAVController::setupEKFEstimator() {
    // Create and configure EKF estimator
    EKFEstimatorConfig ekfConfig;
    ekfConfig.predictionFrequencyHz = 100.0;  // 100 Hz prediction rate
    ekfConfig.positionProcessNoise = 0.01;    // Position process noise
    ekfConfig.velocityProcessNoise = 0.1;     // Velocity process noise
    ekfConfig.positionMeasurementNoise = 0.01; // Position measurement noise
    
    // Update EKF with configuration
    ekfEstimator.updateConfig(ekfConfig);
}

// Start the controller threads
bool UAVController::start() {
    std::lock_guard<std::mutex> lock(controllerMutex);
    
    if (running) {
        std::cerr << "Controller already running" << std::endl;
        return false;
    }    
    // Set running flag
    running = true;
    
    visionThread = std::thread(&UAVController::visionThreadFunction, this);
    ekfThread = std::thread(&UAVController::ekfThreadFunction, this);
    controlThread = std::thread(&UAVController::controlThreadFunction, this);
    displayThread = std::thread(&UAVController::displayThreadFunction, this);    
    return true;
}

// Stop the controller threads
void UAVController::stop() {
    std::lock_guard<std::mutex> lock(controllerMutex);    
    if (!running) {
        return; // Already stopped
    }    
    // Clear running flag
    running = false;    
    UAV::logger().close();
}

// Wait for threads to finish
void UAVController::join() {
    // Wait for threads to finish
    if (visionThread.joinable()) visionThread.join();
    if (ekfThread.joinable()) ekfThread.join();
    if (controlThread.joinable()) controlThread.join();
    if (displayThread.joinable()) displayThread.join();
}

// Check if controller is running
bool UAVController::isRunning() const {
    return running;
}

// Vision thread function
void UAVController::visionThreadFunction() {    
    // Create drone reference frame transformation
    Eigen::Matrix4d droneTransform;
    droneTransform << 0, -1, 0, 0,
                      1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    
    cv::Mat frame;
    ArucoInputFrame inputFrame;
    int frameCount = 0;
    
    while (running) {
        // Target frame time for ~50 Hz
        auto startTime = std::chrono::steady_clock::now();
        
        if (imageSource->getNextFrame(frame)) {
            if (!frame.empty()) {
                // Process frame with ArUco pipeline
                inputFrame.image = frame.clone();
                inputFrame.sequenceId = frameCount++;
                inputFrame.timestamp = std::chrono::steady_clock::now();
                
                ArucoPoseResult result = arucoProcessor.process(inputFrame);
                
                // Apply drone reference frame transformation
                result.applyTransform(droneTransform);
                
                // Share result with other threads
                arucoData.update(result);
                
                // Log frame processing
                UAV::logger().Write("FRAM", "TimeUS,SeqID,ProcTimeMs,FPS", "QIfd",
                                   UAV::logger().getMicroseconds(),
                                   frameCount,
                                   result.processingTimeMs,
                                   result.fps);
                
                // Log ArUco pose if detection is valid
                if (result.detectionValid) {
                    UAV::logger().Write("APOS", "TimeUS,PosX,PosY,PosZ,QuatW,QuatX,QuatY,QuatZ", "Qfffffff",
                                      UAV::logger().getMicroseconds(),
                                      static_cast<float>(result.position.x()),
                                      static_cast<float>(result.position.y()),
                                      static_cast<float>(result.position.z()),
                                      static_cast<float>(result.orientation.w()),
                                      static_cast<float>(result.orientation.x()),
                                      static_cast<float>(result.orientation.y()),
                                      static_cast<float>(result.orientation.z()));
                }
            }
        }
        
        // Compute elapsed time and sleep to maintain ~50 Hz
        auto elapsed = std::chrono::steady_clock::now() - startTime;
        auto targetFrameTime = std::chrono::milliseconds(20); // 50 Hz target
        
        if (elapsed < targetFrameTime) {
            std::this_thread::sleep_for(targetFrameTime - elapsed);
        }
    }
    
    UAV::logger().Write("VTHR", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Vision thread stopped");
}

// EKF thread function
void UAVController::ekfThreadFunction() {
    UAV::logger().Write("ETHR", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "EKF thread started");
    
    // 100Hz timing (10ms period)
    const auto updatePeriod = std::chrono::milliseconds(10);
    auto nextUpdateTime = std::chrono::steady_clock::now();
    
    ArucoPoseResult latestAruco;
    std::chrono::steady_clock::time_point arucoTimestamp;
    int predictionCount = 0;
    int updateCount = 0;
    
    while (running) {
        // Wait until next scheduled update
        std::this_thread::sleep_until(nextUpdateTime);
        auto currentTime = std::chrono::steady_clock::now();
        
        // Check for new ArUco data
        if (arucoData.getData(latestAruco, arucoTimestamp)) {
            if (latestAruco.detectionValid) {
                // Initialize or update EKF with ArUco measurement
                if (!ekfEstimator.isInitialized()) {
                    if (ekfEstimator.initialize(latestAruco, arucoTimestamp)) {
                        UAV::logger().Write("EKFI", "TimeUS,Status", "QZ",
                                          UAV::logger().getMicroseconds(),
                                          "EKF initialized");
                    }
                } else {
                    if (ekfEstimator.update(latestAruco, arucoTimestamp)) {
                        updateCount++;
                        
                        UAV::logger().Write("EKFU", "TimeUS,Count", "QI",
                                          UAV::logger().getMicroseconds(),
                                          updateCount);
                    }
                }
            }
        }
        
        // Run prediction step if initialized
        if (ekfEstimator.isInitialized()) {
            if (ekfEstimator.predict(currentTime)) {
                predictionCount++;
                
                // Get state estimate
                EKFStateResult state = ekfEstimator.getStateEstimate();
                
                // Share state with other threads
                stateData.update(state);
                
                // Log state (at reduced rate for efficiency)
                if (predictionCount % 10 == 0) {
                    UAV::logger().Write("EKFS", "TimeUS,PosX,PosY,PosZ,VelX,VelY,VelZ", "Qffffff",
                                      UAV::logger().getMicroseconds(),
                                      static_cast<float>(state.position.x()),
                                      static_cast<float>(state.position.y()),
                                      static_cast<float>(state.position.z()),
                                      static_cast<float>(state.velocity.x()),
                                      static_cast<float>(state.velocity.y()),
                                      static_cast<float>(state.velocity.z()));
                }
            }
        }
        
        // Schedule next update
        nextUpdateTime += updatePeriod;
        
        // Handle falling behind schedule
        if (nextUpdateTime < std::chrono::steady_clock::now()) {
            // Reset timing to avoid recursive lag
            nextUpdateTime = std::chrono::steady_clock::now() + updatePeriod;
        }
    }
}

// Control thread function
void UAVController::controlThreadFunction() {
    // 50Hz timing (20ms period)
    const auto updatePeriod = std::chrono::milliseconds(20);
    auto nextUpdateTime = std::chrono::steady_clock::now();
    int cycleCount = 0;
    
    while (running) {
        // Wait until next scheduled update
        std::this_thread::sleep_until(nextUpdateTime);
        
        // Execute one cycle of the state machine
        stateMachine.execute();
        cycleCount++;
        
        // Schedule next update
        nextUpdateTime += updatePeriod;
        
        // Handle falling behind schedule
        if (nextUpdateTime < std::chrono::steady_clock::now()) {
            // Reset timing to avoid recursive lag
            nextUpdateTime = std::chrono::steady_clock::now() + updatePeriod;
        }
    }
}

// Display thread function
void UAVController::displayThreadFunction() {
    
    // Create Windows for display
    cv::namedWindow("ArUco Detection", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("EKF Visualization", cv::WINDOW_AUTOSIZE);
    
    ArucoPoseResult arucoPose;
    EKFStateResult ekfState;
    
    while (running) {
        // Create visualization frames
        cv::Mat arucoViz(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat ekfViz(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // Get latest ArUco detection
        bool hasAruco = arucoData.getData(arucoPose);
        
        // Get latest EKF state
        bool hasEKF = stateData.getData(ekfState);
        
        // Show ArUco debug image if available
        if (hasAruco && !arucoPose.debugImage.empty()) {
            arucoViz = arucoPose.debugImage.clone();
        } else {
            // Draw basic info on arucoViz
            cv::putText(arucoViz, "Waiting for ArUco detection...", 
                       cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                       cv::Scalar(0, 255, 255), 2);
        }
        
        // Create EKF visualization
        if (hasEKF && ekfState.valid) {
            // Draw EKF state information
            cv::putText(ekfViz, "EKF State Estimate", 
                       cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                       cv::Scalar(255, 255, 255), 2);
            
            // Position
            cv::putText(ekfViz, 
                       "Position: " + std::to_string(ekfState.position.x()) + ", " +
                       std::to_string(ekfState.position.y()) + ", " +
                       std::to_string(ekfState.position.z()), 
                       cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                       cv::Scalar(0, 255, 0), 2);
            
            // Velocity
            cv::putText(ekfViz, 
                       "Velocity: " + std::to_string(ekfState.velocity.x()) + ", " +
                       std::to_string(ekfState.velocity.y()) + ", " +
                       std::to_string(ekfState.velocity.z()), 
                       cv::Point(20, 110), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                       cv::Scalar(0, 255, 255), 2);
            
            // Position std dev
            cv::putText(ekfViz, 
                       "Position StdDev: " + std::to_string(ekfState.positionStdDev.x()) + ", " +
                       std::to_string(ekfState.positionStdDev.y()) + ", " +
                       std::to_string(ekfState.positionStdDev.z()), 
                       cv::Point(20, 150), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                       cv::Scalar(255, 200, 0), 2);
            
            // Draw 3D visualization (simple top-down view)
            int centerX = 450;
            int centerY = 300;
            int scale = 100; // Pixels per meter
            
            // Draw axes
            cv::line(ekfViz, cv::Point(centerX, centerY), 
                    cv::Point(centerX + scale, centerY), 
                    cv::Scalar(0, 0, 255), 2); // X-axis (red)
                    
            cv::line(ekfViz, cv::Point(centerX, centerY), 
                    cv::Point(centerX, centerY - scale), 
                    cv::Scalar(0, 255, 0), 2); // Y-axis (green)
            
            // Draw origin
            cv::circle(ekfViz, cv::Point(centerX, centerY), 5, 
                      cv::Scalar(255, 255, 255), -1);
            
            // Draw UAV position
            int uavX = centerX + static_cast<int>(ekfState.position.x() * scale);
            int uavY = centerY - static_cast<int>(ekfState.position.y() * scale);
            cv::circle(ekfViz, cv::Point(uavX, uavY), 8, 
                      cv::Scalar(0, 255, 255), -1);
            
            // Draw velocity vector
            cv::arrowedLine(ekfViz, cv::Point(uavX, uavY),
                           cv::Point(uavX + static_cast<int>(ekfState.velocity.x() * scale * 0.5),
                                    uavY - static_cast<int>(ekfState.velocity.y() * scale * 0.5)),
                           cv::Scalar(255, 0, 255), 2);
                           
            // Draw position uncertainty ellipse
            cv::ellipse(ekfViz, cv::Point(uavX, uavY),
                       cv::Size(static_cast<int>(ekfState.positionStdDev.x() * scale),
                               static_cast<int>(ekfState.positionStdDev.y() * scale)),
                       0, 0, 360, cv::Scalar(100, 100, 255), 1);
        } else {
            cv::putText(ekfViz, "Waiting for EKF initialization...", 
                       cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                       cv::Scalar(0, 255, 255), 2);
        }
        
        // Display visualizations
        cv::imshow("ArUco Detection", arucoViz);
        cv::imshow("EKF Visualization", ekfViz);
        
        // Check for key press (ESC to exit)
        int key = cv::waitKey(30);
        if (key == 27) { // ESC key
            // Signal to stop
            {
                std::lock_guard<std::mutex> lock(controllerMutex);
                running = false;
            }
            
            UAV::logger().Write("DTHR", "TimeUS,Status", "QZ",
                               UAV::logger().getMicroseconds(),
                               "User pressed ESC, stopping");
            break;
        }
    }
    
    // Destroy windows
    cv::destroyAllWindows();
    
    UAV::logger().Write("DTHR", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Display thread stopped");
}

// Command the UAV to take off
void UAVController::takeoff(float altitude) {
    if (!running) return;
    // Use the FC to take off
    // Mavlink command to take off
}

// Command the UAV to land
void UAVController::land() {
    if (!running) return;
    // Use the FC to land
    // Mavlink command to land
}

// Execute emergency stop
void UAVController::emergencyStop() {
    if (!running) return;
    // Use the FC to stop
    // Mavlink command to stop
}

// Go to a position
void UAVController::goToPosition(const Eigen::Vector3d& position) {
    if (!running) return;
    
    UAV::logger().Write("GOTO", "TimeUS,X,Y,Z", "Qfff",
                       UAV::logger().getMicroseconds(),
                       static_cast<float>(position.x()),
                       static_cast<float>(position.y()),
                       static_cast<float>(position.z()));
    
    // Set position target
    stateMachine.setPositionTarget(position);
    
    // Ensure we're in position hold mode
    stateMachine.setControlMode(ControlMode::POSITION_CONTROL);
    
    // Set state to hover (which will follow the position target)
    stateMachine.setState(UAVState::HOVER);
}

// Set velocity
void UAVController::setVelocity(const Eigen::Vector3d& velocity) {
    if (!running) return;
    
    UAV::logger().Write("SVEL", "TimeUS,VX,VY,VZ", "Qfff",
                       UAV::logger().getMicroseconds(),
                       static_cast<float>(velocity.x()),
                       static_cast<float>(velocity.y()),
                       static_cast<float>(velocity.z()));
    
    // Set velocity target
    stateMachine.setVelocityTarget(velocity);
    
    // Set control mode to velocity control
    stateMachine.setControlMode(ControlMode::VELOCITY_CONTROL);
}
