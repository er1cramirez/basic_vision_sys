#include "UAVController.h"
#include "ImageSourceFactory.h"
#include "Logger.h"
#include "parameters.h"

#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>
// Note: Removed opencv2/highgui.hpp for headless operation

// Constructor
UAVController::UAVController()
    : running(false),
      httpVisualizationEnabled(false),
      httpVisualizationPort(8888),
      visualizationUpdateRateHz(10),  // Default 10 FPS for visualization
      stateMachine(stateData, controlData) {
          
    // Create log directory and prefix
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << "uav_control_" << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
    logPrefix = ss.str();
    logDirectory = "simLogs";
}

// Destructor
UAVController::~UAVController() {
    // Stop threads if they're running
    if (running) {
        stop();
    }
}

// Initialize the controller
bool UAVController::initialize() {
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
                       "UAV Controller 1.0 (Headless)");
    
    // Create image source
    if (!UAV_Parameters::IS_SIMULATOR) {
        imageSource = ImageSourceFactory::createCameraSource();
        
        UAV::logger().Write("SRCE", "TimeUS,Type", "QZ",
                           UAV::logger().getMicroseconds(),
                           "Camera");
    } else {
        imageSource = ImageSourceFactory::createGazeboSource();
        
        UAV::logger().Write("SRCE", "TimeUS,Type", "QZ",
                           UAV::logger().getMicroseconds(),
                           "Gazebo");
    }
    
    // Initialize image source
    if (!imageSource || !imageSource->initialize()) {
        UAV::logger().Write("ERRR", "TimeUS,Message", "QZ",
                           UAV::logger().getMicroseconds(),
                           "Failed to initialize image source");
        
        std::cerr << "Failed to initialize image source" << std::endl;
        return false;
    }
    
    // Setup ArUco pipeline
    setupArucoPipeline();
    
    // Setup EKF estimator
    setupEKFEstimator();
    
    // Initialize state machine
    if (!stateMachine.initialize()) {
        UAV::logger().Write("ERRR", "TimeUS,Message", "QZ",
                           UAV::logger().getMicroseconds(),
                           "Failed to initialize state machine");
        
        std::cerr << "Failed to initialize state machine" << std::endl;
        return false;
    }
    
    // Initialize HTTP visualization if enabled
    if (httpVisualizationEnabled) {
        debugVisualizer = std::make_unique<GenericDebugVisualizer>(httpVisualizationPort);
        debugVisualizer->setJpegQuality(30);
        debugVisualizer->setMaxFrameSize(640, 480);
        
        UAV::logger().Write("HTTP", "TimeUS,Port,Status", "QIZ",
                           UAV::logger().getMicroseconds(),
                           httpVisualizationPort,
                           "HTTP visualizer initialized");
        
        std::cout << "HTTP visualization will be available at: http://localhost:" 
                  << httpVisualizationPort << std::endl;
    } else {
        UAV::logger().Write("HTTP", "TimeUS,Status", "QZ",
                           UAV::logger().getMicroseconds(),
                           "HTTP visualizer disabled - running headless");
        
        std::cout << "Running in headless mode (no visualization)" << std::endl;
    }
    
    UAV::logger().Write("INIT", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Controller initialized successfully");

    // Initialize MAVLink communication
    if (UAV_Parameters::IS_SIMULATOR) {
        try {
            mavlinkModule = std::make_unique<MavlinkCommModule>(
                UAV_Parameters::UDP_IP, UAV_Parameters::GZ_MAV_PORT, 
                UAV_Parameters::SYS_ID, UAV_Parameters::COMP_ID,
                UAV_Parameters::TG_ID, UAV_Parameters::TG_COMP);
            
            mavlinkModule->setFrequency(UAV_Parameters::COM_FREQ);
            
            UAV::logger().Write("MAVL", "TimeUS,IP,Port", "QZI",
                               UAV::logger().getMicroseconds(),
                               UAV_Parameters::UDP_IP.c_str(),
                               UAV_Parameters::GZ_MAV_PORT);
        } catch (const std::exception& e) {
            UAV::logger().Write("ERRR", "TimeUS,Message", "QZ",
                               UAV::logger().getMicroseconds(),
                               e.what());
            return false;
        }
    } else {
        try {
            mavlinkModule = std::make_unique<MavlinkCommModule>(
                UAV_Parameters::MAV_SER_DEV, UAV_Parameters::SER_BAUD,
                UAV_Parameters::SYS_ID, UAV_Parameters::COMP_ID,
                UAV_Parameters::TG_ID, UAV_Parameters::TG_COMP, true);
            
            mavlinkModule->setFrequency(UAV_Parameters::COM_FREQ);
            
            UAV::logger().Write("MAVL", "TimeUS,Device,BaudRate", "QZI",
                               UAV::logger().getMicroseconds(),
                               UAV_Parameters::MAV_SER_DEV.c_str(),
                               UAV_Parameters::SER_BAUD);
        } catch (const std::exception& e) {
            UAV::logger().Write("ERRR", "TimeUS,Message", "QZ",
                               UAV::logger().getMicroseconds(),
                               e.what());
            return false;
        }
    }
    
    return true;
}

// Configure ArUco pipeline
void UAVController::setupArucoPipeline() {
    ArucoPoseSettings settings;
    settings.dictionaryId = cv::aruco::DICT_5X5_1000;
    settings.markerSizeMeters = UAV_Parameters::ARUCO_MARKER_SIZE;
    settings.useCornerRefinement = true;
    settings.cornerRefinementMaxIterations = 30;
    settings.cornerRefinementMinAccuracy = 0.01;
    settings.maxReprojectionError = 2.0;
    settings.debugVisualization = httpVisualizationEnabled; // Only enable if HTTP viz is on
    
    arucoProcessor.updateSettings(settings);
    
    // Camera calibration
    CameraCalibration calibration;
    if (UAV_Parameters::IS_SIMULATOR) {
        calibration.cameraMatrix = UAV_Parameters::CAM_MAT_SIM;
        calibration.distCoeffs = UAV_Parameters::DIST_COEF_SIM;
    } else {
        calibration.cameraMatrix = UAV_Parameters::CAM_MAT_REAl;
        calibration.distCoeffs = UAV_Parameters::DIST_COEF_REAL;
    }
    
    arucoProcessor.setCalibration(calibration);
    
    UAV::logger().Write("ACFG", "TimeUS,DictID,MarkerSize,UseCornerRef,MaxError", "QIfBf",
                       UAV::logger().getMicroseconds(),
                       settings.dictionaryId,
                       settings.markerSizeMeters,
                       settings.useCornerRefinement ? 1 : 0,
                       settings.maxReprojectionError);
}

// Configure EKF estimator
void UAVController::setupEKFEstimator() {
    // EKF configuration is handled with defaults for now
    // Could be extended to read from parameters file
}

// Start the controller threads
bool UAVController::start() {
    std::lock_guard<std::mutex> lock(controllerMutex);
    
    if (running) {
        std::cerr << "Controller already running" << std::endl;
        return false;
    }
    
    running = true;
    
    // Start HTTP visualizer if enabled
    if (httpVisualizationEnabled && debugVisualizer) {
        debugVisualizer->start();
        debugVisualizer->setStatus("System Starting...");
        
        UAV::logger().Write("HTTP", "TimeUS,Status", "QZ",
                           UAV::logger().getMicroseconds(),
                           "HTTP visualizer started");
    }
    
    // Start MAVLink module if it exists
    if (mavlinkModule) {
        if (!mavlinkModule->start()) {
            UAV::logger().Write("ERRR", "TimeUS,Message", "QZ",
                               UAV::logger().getMicroseconds(),
                               "Failed to start MAVLink communication");
            return false;
        }
        
        UAV::logger().Write("MAVL", "TimeUS,Status", "QZ",
                           UAV::logger().getMicroseconds(),
                           "MAVLink communication started");
    }
    
    // Start threads
    UAV::logger().Write("STRT", "TimeUS,Message", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Starting controller threads");
    
    visionThread = std::thread(&UAVController::visionThreadFunction, this);
    ekfThread = std::thread(&UAVController::ekfThreadFunction, this);
    controlThread = std::thread(&UAVController::controlThreadFunction, this);
    if (httpVisualizationEnabled) {
        visualizationThread = std::thread(&UAVController::visualizationThreadFunction, this);
    }
    
    // Update visualization status
    if (debugVisualizer) {
        debugVisualizer->setStatus("System Running");
    }
    
    return true;
}

// Stop the controller threads
void UAVController::stop() {
    std::lock_guard<std::mutex> lock(controllerMutex);
    
    if (!running) {
        return;
    }
    
    running = false;
    
    UAV::logger().Write("STOP", "TimeUS,Message", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Stopping controller threads");
    
    // Update visualization status
    if (debugVisualizer) {
        debugVisualizer->setStatus("System Stopping...");
    }
    
    // Wait for threads to finish
    join();
    
    // Stop HTTP visualizer
    if (debugVisualizer) {
        debugVisualizer->stop();
        std::cout << "HTTP visualizer stopped" << std::endl;
        UAV::logger().Write("HTTP", "TimeUS,Status", "QZ",
                           UAV::logger().getMicroseconds(),
                           "HTTP visualizer stopped");
    }
    
    // Stop MAVLink
    if (mavlinkModule) {
        mavlinkModule->stop();
        std::cout << "MAVLink communication stopped" << std::endl;
        UAV::logger().Write("MAVL", "TimeUS,Status", "QZ",
                           UAV::logger().getMicroseconds(),
                           "MAVLink communication stopped");
    } else {
        std::cout << "No MAVLink communication to stop" << std::endl;
    }
    
    UAV::logger().Write("EXIT", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Normal shutdown");
    
    UAV::logger().close();
}

// Wait for threads to finish
void UAVController::join() {
    if (visionThread.joinable()) visionThread.join();
    if (ekfThread.joinable()) ekfThread.join();
    if (controlThread.joinable()) controlThread.join();
    if (visualizationThread.joinable()) visualizationThread.join();

}

// Check if controller is running
bool UAVController::isRunning() const {
    return running;
}

// Set HTTP visualization
void UAVController::setHttpVisualization(bool enabled, int port) {
    httpVisualizationEnabled = enabled;
    httpVisualizationPort = port;
}

// Get visualization port
int UAVController::getVisualizationPort() const {
    return httpVisualizationEnabled ? httpVisualizationPort : 0;
}

// Vision thread function
// Optimized vision thread function for UAVController.cpp
void UAVController::visionThreadFunction() {
    UAV::logger().Write("VTHR", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Vision thread started");
    
    // Configure camera for optimal performance if it's a CameraSource
    auto cameraSource = std::dynamic_pointer_cast<CameraSource>(imageSource);
    if (cameraSource) {
        // Disable frame copying for maximum performance
        cameraSource->setShouldCopyFrame(false);
        
        // If you only need grayscale for ArUco, enable preprocessing
        // cameraSource->setPreprocessType(FramePreprocessType::GRAYSCALE);
        
        UAV::logger().Write("VCFG", "TimeUS,OptimizationsEnabled", "QZ",
                           UAV::logger().getMicroseconds(),
                           "Zero-copy frame acquisition");
    }
    
    Eigen::Matrix4d droneTransform;
    droneTransform << 0, -1, 0, 0,
                      1, 0, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;
    
    // Performance tracking
    auto fpsStartTime = std::chrono::steady_clock::now();
    int fpsFrameCount = 0;
    double currentFps = 0.0;
    
    cv::Mat frame;
    ArucoInputFrame inputFrame;
    int frameCount = 0;
    
    // Pre-allocate visualization data to avoid repeated allocations
    VisualizationData vizUpdate;
    
    while (running) {
        auto startTime = std::chrono::steady_clock::now();
        
        if (imageSource->getNextFrame(frame)) {
            if (!frame.empty()) {
                // CRITICAL OPTIMIZATION: Avoid frame.clone() here!
                // Instead, let ArUco processor handle the frame directly
                inputFrame.image = frame;  // Shallow copy (just reference)
                inputFrame.sequenceId = frameCount++;
                inputFrame.timestamp = std::chrono::steady_clock::now();
                
                // Process frame with ArUco pipeline
                ArucoPoseResult result = arucoProcessor.process(inputFrame);
                
                // Apply drone reference frame transformation
                result.applyTransform(droneTransform);
                
                // Share result with other threads
                arucoData.update(result);
                
                // Update visualization data only if enabled
                if (httpVisualizationEnabled && !result.debugImage.empty()) {
                    // Reuse the vizUpdate object to avoid allocations
                    vizUpdate.debugImage = result.debugImage;  // Direct assignment
                    vizUpdate.frameId = frameCount;
                    vizUpdate.visionFps = currentFps;
                    vizUpdate.timestamp = std::chrono::steady_clock::now();
                    
                    // Add ArUco-specific data
                    vizUpdate.setStatus("ArUco_Valid", result.detectionValid);
                    if (result.detectionValid) {
                        vizUpdate.setStatus("ArUco_X", result.position.x());
                        vizUpdate.setStatus("ArUco_Y", result.position.y());
                        vizUpdate.setStatus("ArUco_Z", result.position.z());
                    }
                    
                    vizData.update(vizUpdate);
                }
                
                // Update FPS calculation
                fpsFrameCount++;
                auto fpsElapsed = std::chrono::steady_clock::now() - fpsStartTime;
                if (fpsElapsed >= std::chrono::seconds(1)) {
                    currentFps = fpsFrameCount / std::chrono::duration<double>(fpsElapsed).count();
                    fpsFrameCount = 0;
                    fpsStartTime = std::chrono::steady_clock::now();
                    
                    // Log performance metrics
                    // UAV::logger().Write("VPERF", "TimeUS,FPS,FrameAcqMs,ProcessMs", "Qfff",
                    //                    UAV::logger().getMicroseconds(),
                    //                    static_cast<float>(currentFps),
                    //                    static_cast<float>(result.frameAcquisitionTimeMs),
                    //                    static_cast<float>(result.processingTimeMs));
                }
            }
        }
        
        // Remove sleep - let the camera's frame rate control the loop timing
        // The getNextFrame() call will naturally pace the loop
        
        // Only add a small sleep if we're running way too fast (unlikely with camera)
        auto elapsed = std::chrono::steady_clock::now() - startTime;
        if (elapsed < std::chrono::milliseconds(2)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    UAV::logger().Write("VTHR", "TimeUS,Status,TotalFrames", "QZI",
                       UAV::logger().getMicroseconds(),
                       "Vision thread stopped",
                       frameCount);
}

// New visualization thread function
void UAVController::visualizationThreadFunction() {
    UAV::logger().Write("VIZT", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Visualization thread started");
    
    const auto updatePeriod = std::chrono::milliseconds(1000 / visualizationUpdateRateHz);
    auto nextUpdateTime = std::chrono::steady_clock::now();
    
    while (running) {
        std::this_thread::sleep_until(nextUpdateTime);
        
        // Get latest visualization data
        VisualizationData latestViz;
        if (vizData.getData(latestViz)) {
            // Update frame if available
            if (!latestViz.debugImage.empty()) {
                debugVisualizer->updateFrame(latestViz.debugImage, latestViz.frameId);
            }
            
            // Update all status data
            for (const auto& [key, value] : latestViz.statusData) {
                debugVisualizer->setData(key, value);
            }
            
            // Update system metrics
            debugVisualizer->setData("Vision_FPS", latestViz.visionFps);
            debugVisualizer->setData("EKF_FPS", latestViz.ekfFps);
            debugVisualizer->setData("Control_FPS", latestViz.controlFps);
        }
        
        // Also pull latest data from other sources
        EKFStateResult ekfState;
        if (stateData.getData(ekfState) && ekfState.valid) {
            debugVisualizer->setData("EKF_PosX", ekfState.position.x());
            debugVisualizer->setData("EKF_PosY", ekfState.position.y());
            debugVisualizer->setData("EKF_PosZ", ekfState.position.z());
            debugVisualizer->setData("EKF_VelX", ekfState.velocity.x());
            debugVisualizer->setData("EKF_VelY", ekfState.velocity.y());
            debugVisualizer->setData("EKF_VelZ", ekfState.velocity.z());
        }
        
        ControlOutput control;
        if (controlData.getData(control)) {
            debugVisualizer->setData("Control_Fx", control.u_desired.x());
            debugVisualizer->setData("Control_Fy", control.u_desired.y());
            debugVisualizer->setData("Control_Fz", control.u_desired.z());
        }
        
        // Update system status
        debugVisualizer->setData("State", std::to_string(static_cast<int>(stateMachine.getState())));
        debugVisualizer->setData("Mode", std::to_string(static_cast<int>(stateMachine.getControlMode())));
        
        nextUpdateTime += updatePeriod;
        
        // Handle timing overruns
        if (nextUpdateTime < std::chrono::steady_clock::now()) {
            nextUpdateTime = std::chrono::steady_clock::now() + updatePeriod;
        }
    }
    
    UAV::logger().Write("VIZT", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Visualization thread stopped");
}

// EKF thread function
void UAVController::ekfThreadFunction() {
    UAV::logger().Write("ETHR", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "EKF thread started");
    
    const auto updatePeriod = std::chrono::milliseconds(10); // 100Hz
    auto nextUpdateTime = std::chrono::steady_clock::now();
    
    ArucoPoseResult latestAruco;
    std::chrono::steady_clock::time_point arucoTimestamp;
    int predictionCount = 0;
    int updateCount = 0;
    
    while (running) {
        std::this_thread::sleep_until(nextUpdateTime);
        auto currentTime = std::chrono::steady_clock::now();
        
        // Check for new ArUco data
        if (arucoData.getData(latestAruco, arucoTimestamp)) {
            if (latestAruco.detectionValid) {
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
                
                EKFStateResult state = ekfEstimator.getStateEstimate();
                stateData.update(state);
                
                // Update HTTP visualizer if enabled
                if (debugVisualizer) {
                    // debugVisualizer->updateEKFState(state);
                }
                
                // Log state (at reduced rate)
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
        
        nextUpdateTime += updatePeriod;
        
        if (nextUpdateTime < std::chrono::steady_clock::now()) {
            UAV::logger().Write("EWRN", "TimeUS,Warning", "QZ",
                              UAV::logger().getMicroseconds(),
                              "EKF thread falling behind schedule");
            nextUpdateTime = std::chrono::steady_clock::now() + updatePeriod;
        }
    }
    
    UAV::logger().Write("ETHR", "TimeUS,Status,PredCount,UpdateCount", "QZII",
                       UAV::logger().getMicroseconds(),
                       "EKF thread stopped",
                       predictionCount,
                       updateCount);
}

// Control thread function
void UAVController::controlThreadFunction() {
    UAV::logger().Write("CTHR", "TimeUS,Status", "QZ",
                       UAV::logger().getMicroseconds(),
                       "Control thread started");
    
    const auto updatePeriod = std::chrono::milliseconds(10); // 100Hz
    auto nextUpdateTime = std::chrono::steady_clock::now();
    int cycleCount = 0;
    
    while (running) {
        std::this_thread::sleep_until(nextUpdateTime);
        
        // Execute one cycle of the state machine
        stateMachine.execute();
        cycleCount++;

        if (mavlinkModule) {
            ControlOutput control;
            if (controlData.getData(control)) {
                // Update MAVLink module with control output
                mavlinkModule->setForceVector(
                    static_cast<float>(control.u_desired.x()),
                    static_cast<float>(control.u_desired.y()),
                    static_cast<float>(control.u_desired.z()),
                    static_cast<float>(control.u_desired_dot.x()),
                    static_cast<float>(control.u_desired_dot.y()),
                    static_cast<float>(control.u_desired_dot.z())
                );
                
                // Update HTTP visualizer if enabled
                // if (debugVisualizer) {
                //     debugVisualizer->updateControlOutput(control);
                // }
                
                // Log force vector (every 10th update)
                if (cycleCount % 10 == 0) {
                    UAV::logger().Write("MAVF", "TimeUS,Fx,Fy,Fz", "Qfff",
                                      UAV::logger().getMicroseconds(),
                                      static_cast<float>(control.u_desired.x()),
                                      static_cast<float>(control.u_desired.y()),
                                      static_cast<float>(control.u_desired.z()));
                }
            }
        }
        
        nextUpdateTime += updatePeriod;
        
        if (nextUpdateTime < std::chrono::steady_clock::now()) {
            UAV::logger().Write("CWRN", "TimeUS,Warning", "QZ",
                              UAV::logger().getMicroseconds(),
                              "Control thread falling behind schedule");
            nextUpdateTime = std::chrono::steady_clock::now() + updatePeriod;
        }
    }
    
    UAV::logger().Write("CTHR", "TimeUS,Status,CycleCount", "QZI",
                       UAV::logger().getMicroseconds(),
                       "Control thread stopped",
                       cycleCount);
}