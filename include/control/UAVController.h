#ifndef UAV_CONTROLLER_H
#define UAV_CONTROLLER_H

#include "LatestData.h"
#include "UAVStateMachine.h"
#include "aruco_pose_pipeline.h"
#include "aruco_ekf_estimator.h"
#include "Logger.h"
#include "ImageSourceInterface.h"
#include "mavlink_comm_module.h"
#include "GenericDebugVisualizer.h" 
#include "VisualizationData.h"

#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include <mutex>

/**
 * @class UAVController
 * @brief Main controller class for the UAV system
 * 
 * This class manages the different threads and components of the UAV system.
 * Now supports both headless operation and remote HTTP-based visualization.
 */
class UAVController {
public:
    /**
     * @brief Constructor
     */
    UAVController();
    
    /**
     * @brief Destructor
     */
    ~UAVController();
    
    /**
     * @brief Initialize the controller
     * @return True if initialization succeeded
     */
    bool initialize();
    
    /**
     * @brief Start the controller threads
     * @return True if start succeeded
     */
    bool start();
    
    /**
     * @brief Stop the controller threads
     */
    void stop();
    
    /**
     * @brief Wait for controller to finish
     */
    void join();
    
    /**
     * @brief Check if controller is running
     * @return True if running
     */
    bool isRunning() const;

    /**
     * @brief Enable/disable remote HTTP visualization
     * @param enabled True to enable HTTP server visualization
     * @param port HTTP server port (default 8888)
     */
    void setHttpVisualization(bool enabled, int port = 8888);
    
    /**
     * @brief Get the HTTP visualization port
     * @return Port number if HTTP visualization is enabled, 0 otherwise
     */
    int getVisualizationPort() const;

private:
    // Thread synchronization
    std::atomic<bool> running;
    std::mutex controllerMutex;

    // Thread handles
    std::thread visionThread;
    std::thread ekfThread;
    std::thread controlThread;
    std::thread visualizationThread;  // NEW: Separate visualization thread
    
    // Thread data exchange
    LatestData<ArucoPoseResult> arucoData;
    LatestData<EKFStateResult> stateData;
    LatestData<ControlOutput> controlData;
    LatestData<VisualizationData> vizData;  // NEW: Visualization data exchange
    
    // Visualization control
    bool httpVisualizationEnabled;
    int httpVisualizationPort;
    int visualizationUpdateRateHz;  // NEW: Configurable update rate
    
    
    // Thread functions
    void visionThreadFunction();
    void ekfThreadFunction();
    void controlThreadFunction();
    void visualizationThreadFunction();  // NEW: Visualization thread function

    
    // System components
    std::unique_ptr<MavlinkCommModule> mavlinkModule;
    std::unique_ptr<GenericDebugVisualizer> debugVisualizer;
    ImageSourcePtr imageSource;
    ArucoPosePipeline arucoProcessor;
    ArucoEKFEstimator ekfEstimator;
    UAVStateMachine stateMachine;
    
    // Configuration
    std::string logDirectory;
    std::string logPrefix;
    
    
    // Helper functions
    bool initializeMavlink();
    void setupArucoPipeline();
    void setupEKFEstimator();
    
};

#endif // UAV_CONTROLLER_H