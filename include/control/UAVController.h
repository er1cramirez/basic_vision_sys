#ifndef UAV_CONTROLLER_H
#define UAV_CONTROLLER_H

#include "LatestData.h"
#include "UAVStateMachine.h"
#include "aruco_pose_pipeline.h"
#include "aruco_ekf_estimator.h"
#include "Logger.h"
#include "ImageSourceInterface.h"
#include "mavlink_comm_module.h"
#include "GenericDebugVisualizer.h"  // No dependencies!

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
    // Visualization control
    bool httpVisualizationEnabled;
    int httpVisualizationPort;
    std::unique_ptr<GenericDebugVisualizer> debugVisualizer;
    
    // Thread synchronization
    std::atomic<bool> running;
    std::mutex controllerMutex;
    
    // Thread handles
    std::thread visionThread;
    std::thread ekfThread;
    std::thread controlThread;
    // Note: displayThread removed - replaced with HTTP server
    
    // Thread data exchange
    LatestData<ArucoPoseResult> arucoData;
    LatestData<EKFStateResult> stateData;
    LatestData<ControlOutput> controlData;
    
    // System components
    std::unique_ptr<MavlinkCommModule> mavlinkModule;
    ImageSourcePtr imageSource;
    ArucoPosePipeline arucoProcessor;
    ArucoEKFEstimator ekfEstimator;
    UAVStateMachine stateMachine;
    
    // Configuration
    std::string logDirectory;
    std::string logPrefix;
    
    // Thread functions
    void visionThreadFunction();
    void ekfThreadFunction();
    void controlThreadFunction();
    
    // Helper functions
    void setupArucoPipeline();
    void setupEKFEstimator();
    
    // Visualization helpers
    void updateVisualization();
};

#endif // UAV_CONTROLLER_H