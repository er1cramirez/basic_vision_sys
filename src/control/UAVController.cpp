#include "UAVController.h"
#include "ImageSourceFactory.h"
#include "Logger.h"
#include "parameters.h"
#include "SimpleTelemetry.h"

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
    
    // Shutdown telemetry
    SimpleTelemetry::shutdown();
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
    
    // Initialize telemetry system
    if (!SimpleTelemetry::initialize(14559)) {
        std::cerr << "Warning: Failed to initialize telemetry system" << std::endl;
        // Continue anyway, telemetry is optional
    } else {
        std::cout << "Telemetry system initialized on port 14559" << std::endl;
    }
    // Create image source
    if (!UAV_Parameters::IS_SIMULATOR) {
        imageSource = ImageSourceFactory::createCameraSource();
    } else {
        imageSource = ImageSourceFactory::createGazeboSource();
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
    
    // Initialize HTTP visualization if enabled
    if (httpVisualizationEnabled) {
        debugVisualizer = std::make_unique<GenericDebugVisualizer>(httpVisualizationPort);
        debugVisualizer->setJpegQuality(30);
        debugVisualizer->setMaxFrameSize(640, 480);
        std::cout << "HTTP visualization will be available at: http://localhost:" 
                  << httpVisualizationPort << std::endl;
    } else {
        std::cout << "Running in headless mode (no visualization)" << std::endl;
    }
    // Initialize MAVLink communication
    if (UAV_Parameters::IS_SIMULATOR) {
        try {
            mavlinkModule = std::make_unique<MavlinkCommModule>(
                UAV_Parameters::UDP_IP, UAV_Parameters::GZ_MAV_PORT, 
                UAV_Parameters::SYS_ID, UAV_Parameters::COMP_ID,
                UAV_Parameters::TG_ID, UAV_Parameters::TG_COMP);   
            mavlinkModule->setFrequency(UAV_Parameters::COM_FREQ);
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize MAVLink communication: " << e.what() << std::endl;
            return false;
        }
    } else {
        try {
            mavlinkModule = std::make_unique<MavlinkCommModule>(
                UAV_Parameters::MAV_SER_DEV, UAV_Parameters::SER_BAUD,
                UAV_Parameters::SYS_ID, UAV_Parameters::COMP_ID,
                UAV_Parameters::TG_ID, UAV_Parameters::TG_COMP, true);
            
            mavlinkModule->setFrequency(UAV_Parameters::COM_FREQ);
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize MAVLink communication: " << e.what() << std::endl;
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
    }
    
    // Start MAVLink module if it exists
    if (mavlinkModule) {
        if (!mavlinkModule->start()) {
            std::cerr << "Failed to start MAVLink communication" << std::endl;
            return false;
        }
        std::cout << "MAVLink communication started" << std::endl;
    }
    
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
    }
    
    // Stop MAVLink
    if (mavlinkModule) {
        mavlinkModule->stop();
        std::cout << "MAVLink communication stopped" << std::endl;
    } else {
        std::cout << "No MAVLink communication to stop" << std::endl;
    }
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
void UAVController::visionThreadFunction() {
    Eigen::Matrix4d droneTransform;
        droneTransform << 0, -1, 0, 0,
                        1, 0, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
    // Performance tracking
    auto fpsStartTime = std::chrono::steady_clock::now();
    int fpsFrameCount = 0;
    double currentFps = 0.0;
    
    // Quaternion debugging variables
    auto lastQuaternionPrintTime = std::chrono::steady_clock::now();
    const auto quaternionPrintInterval = std::chrono::seconds(2); // Print every 2 seconds
    
    cv::Mat frame;
    ArucoInputFrame inputFrame;
    int frameCount = 0;
    
    while (running) {
        auto startTime = std::chrono::steady_clock::now();
        
        if (imageSource->getNextFrame(frame)) {
            if (!frame.empty()) {
                // Process frame with ArUco pipeline
                inputFrame.image = frame.clone();
                inputFrame.sequenceId = frameCount++;
                inputFrame.timestamp = std::chrono::steady_clock::now();
                
                ArucoPoseResult result = arucoProcessor.process(inputFrame);

                // Log aruco position if valid
                if (result.detectionValid) {
                    UAV::logger().Write("ARUC", "TimeUs,SeqID,PosX,PosY,PosZ,Valid", "QQfffb",
                                       UAV::logger().getMicroseconds(),
                                       inputFrame.sequenceId,
                                       result.position.x(),
                                       result.position.y(),
                                       result.position.z(),
                                       result.detectionValid);
                } 
                
                // Apply drone reference frame transformation
                result.applyTransform(droneTransform);

                // Log Aruco after transformation
                UAV::logger().Write("POST", "TimeUs,SeqID,PosX,PosY,PosZ,Valid", "QQfffb",
                                   UAV::logger().getMicroseconds(),
                                   inputFrame.sequenceId,
                                   result.position.x(),
                                   result.position.y(),
                                   result.position.z(),
                                   result.detectionValid);
                
                // Share result with other threads
                arucoData.update(result);
                
                // Update visualization data (FAST - no frame cloning here!)
                if (httpVisualizationEnabled) {
                    VisualizationData vizUpdate;
                    vizUpdate.debugImage = result.debugImage;  // Direct assignment, no clone
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
                }
                
                // Log frame processing
                UAV::logger().Write("FRAM", "TimeUS,SeqID,ProcTimeMs,FPS", "QIfd",
                                   UAV::logger().getMicroseconds(),
                                   frameCount,
                                   result.processingTimeMs,
                                   result.fps);
            }
        }
        
        // Read and debug quaternion data from MAVLink
        if (mavlinkModule) {
            float q1, q2, q3, q4;
            auto currentTime = std::chrono::steady_clock::now();
            
            if (mavlinkModule->getAttitudeQuaternion(q1, q2, q3, q4)) {
                // Log quaternion data for detailed analysis
                UAV::logger().Write("QUAT", "TimeUs,Q1,Q2,Q3,Q4,Timestamp", "Qfffff",
                                   UAV::logger().getMicroseconds(),
                                   q1, q2, q3, q4,
                                   static_cast<float>(mavlinkModule->getQuaternionTimestamp()));
                
                // Debug print quaternion periodically
                if (currentTime - lastQuaternionPrintTime >= quaternionPrintInterval) {
                    std::cout << "[VISION] Attitude Quaternion: "
                              << "w=" << std::fixed << std::setprecision(4) << q1 << " "
                              << "x=" << std::fixed << std::setprecision(4) << q2 << " "
                              << "y=" << std::fixed << std::setprecision(4) << q3 << " "
                              << "z=" << std::fixed << std::setprecision(4) << q4
                              << " (timestamp: " << mavlinkModule->getQuaternionTimestamp() << ")"
                              << std::endl;
                    lastQuaternionPrintTime = currentTime;
                }
                
                // Update visualization with quaternion data if enabled
                if (httpVisualizationEnabled) {
                    // These will be available in the web interface
                    VisualizationData quatVizUpdate;
                    quatVizUpdate.setStatus("Quat_W", q1);
                    quatVizUpdate.setStatus("Quat_X", q2);
                    quatVizUpdate.setStatus("Quat_Y", q3);
                    quatVizUpdate.setStatus("Quat_Z", q4);
                    quatVizUpdate.setStatus("Quat_Valid", true);
                    quatVizUpdate.timestamp = currentTime;
                    vizData.update(quatVizUpdate);
                }
            } else {
                // Debug print when no valid quaternion data
                if (currentTime - lastQuaternionPrintTime >= quaternionPrintInterval) {
                    std::cout << "[VISION] No valid attitude quaternion data available" << std::endl;
                    lastQuaternionPrintTime = currentTime;
                }
                
                // Update visualization to show invalid data
                if (httpVisualizationEnabled) {
                    VisualizationData quatVizUpdate;
                    quatVizUpdate.setStatus("Quat_Valid", false);
                    quatVizUpdate.timestamp = currentTime;
                    vizData.update(quatVizUpdate);
                }
            }
        }
        
        // Maintain target frame rate
        auto elapsed = std::chrono::steady_clock::now() - startTime;
        auto targetFrameTime = std::chrono::milliseconds(20); // ~50 Hz
        
        if (elapsed < targetFrameTime) {
            std::this_thread::sleep_for(targetFrameTime - elapsed);
        }
    }
}

// New visualization thread function
void UAVController::visualizationThreadFunction() {    
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
}

// EKF thread function
void UAVController::ekfThreadFunction() {    
    const auto updatePeriod = std::chrono::milliseconds(20); // 50Hz
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
                        std::cout << "EKF initialized with first ArUco measurement" << std::endl;
                    }
                } else {
                    if (ekfEstimator.update(latestAruco, arucoTimestamp)) {
                        updateCount++;
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
                
                // Send telemetry data
                if (state.valid) {
                    // Position data
                    TEL_VECTOR("ekf/position", state.position.x(), state.position.y(), state.position.z());
                    TEL_NUMBER("ekf/position_x", state.position.x());
                    TEL_NUMBER("ekf/position_y", state.position.y());
                    TEL_NUMBER("ekf/position_z", state.position.z());
                    TEL_NUMBER("ekf/position_magnitude", state.position.norm());
                    
                    // Velocity data
                    TEL_VECTOR("ekf/velocity", state.velocity.x(), state.velocity.y(), state.velocity.z());
                    TEL_NUMBER("ekf/velocity_x", state.velocity.x());
                    TEL_NUMBER("ekf/velocity_y", state.velocity.y());
                    TEL_NUMBER("ekf/velocity_z", state.velocity.z());
                    TEL_NUMBER("ekf/velocity_magnitude", state.velocity.norm());
                    
                    // Acceleration data
                    TEL_VECTOR("ekf/acceleration", state.acceleration.x(), state.acceleration.y(), state.acceleration.z());
                    TEL_NUMBER("ekf/acceleration_x", state.acceleration.x());
                    TEL_NUMBER("ekf/acceleration_y", state.acceleration.y());
                    TEL_NUMBER("ekf/acceleration_z", state.acceleration.z());
                    TEL_NUMBER("ekf/acceleration_magnitude", state.acceleration.norm());
                    
                    // Uncertainty data
                    TEL_VECTOR("ekf/position_stddev", state.positionStdDev.x(), state.positionStdDev.y(), state.positionStdDev.z());
                    TEL_NUMBER("ekf/position_uncertainty", state.positionStdDev.norm());
                    TEL_VECTOR("ekf/velocity_stddev", state.velocityStdDev.x(), state.velocityStdDev.y(), state.velocityStdDev.z());
                    TEL_NUMBER("ekf/velocity_uncertainty", state.velocityStdDev.norm());
                    TEL_VECTOR("ekf/acceleration_stddev", state.accelerationStdDev.x(), state.accelerationStdDev.y(), state.accelerationStdDev.z());
                    TEL_NUMBER("ekf/acceleration_uncertainty", state.accelerationStdDev.norm());
                    
                    // Status indicators
                    TEL_BOOL("ekf/valid", state.valid);
                    TEL_NUMBER("ekf/prediction_count", predictionCount);
                }
                
                // Update HTTP visualizer if enabled
                if (debugVisualizer) {
                    // debugVisualizer->updateEKFState(state);
                }
                
                // Additional system telemetry
                TEL_NUMBER("system/ekf_update_count", updateCount);
                TEL_NUMBER("system/ekf_prediction_count", predictionCount);
                TEL_BOOL("system/ekf_initialized", ekfEstimator.isInitialized());
                
                // Log state (at reduced rate)
                if (predictionCount % 10 == 0) {
                    UAV::logger().Write("EKFS", "TimeUS,PosX,PosY,PosZ,VelX,VelY,VelZ,AccX,AccY,AccZ", "Qfffffffff",
                                      UAV::logger().getMicroseconds(),
                                      static_cast<float>(state.position.x()),
                                      static_cast<float>(state.position.y()),
                                      static_cast<float>(state.position.z()),
                                      static_cast<float>(state.velocity.x()),
                                      static_cast<float>(state.velocity.y()),
                                      static_cast<float>(state.velocity.z()),
                                      static_cast<float>(state.acceleration.x()),
                                      static_cast<float>(state.acceleration.y()),
                                      static_cast<float>(state.acceleration.z()));
                }
            }
        }
        
        nextUpdateTime += updatePeriod;
        
        if (nextUpdateTime < std::chrono::steady_clock::now()) {
            nextUpdateTime = std::chrono::steady_clock::now() + updatePeriod;
        }
    }
}

// Control thread function
void UAVController::controlThreadFunction() {
    const auto updatePeriod = std::chrono::milliseconds(20); // 50Hz
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
                    UAV::logger().Write("CTRL", "TimeUS,Fx,Fy,Fz", "Qfff",
                                      UAV::logger().getMicroseconds(),
                                      static_cast<float>(control.u_desired.x()),
                                      static_cast<float>(control.u_desired.y()),
                                      static_cast<float>(control.u_desired.z()));
                }
            }
        }
        
        nextUpdateTime += updatePeriod;
        
        if (nextUpdateTime < std::chrono::steady_clock::now()) {
            nextUpdateTime = std::chrono::steady_clock::now() + updatePeriod;
        }
    }
}