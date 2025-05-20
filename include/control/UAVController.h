#ifndef UAV_CONTROLLER_H
#define UAV_CONTROLLER_H

#include "LatestData.h"
#include "UAVStateMachine.h"
#include "aruco_pose_pipeline.h"
#include "aruco_ekf_estimator.h"
#include "Logger.h"
#include "ImageSourceInterface.h" // Use your existing interface

#include "mavlink_comm_module.h"

#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include <mutex>

/**
 * @class UAVController
 * @brief Main controller class for the UAV system
 * 
 * This class manages the different threads and components of the UAV system
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
     * @param gazeboTopic Optional topic for Gazebo camera
     * @param useCamera Use physical camera instead of Gazebo
     * @return True if initialization succeeded
     */
    bool initialize(const std::string& gazeboTopic = "", bool useCamera = false);
    
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
     * @brief Command the UAV to take off
     * @param altitude Target altitude for takeoff
     */
    void takeoff(float altitude);
    
    /**
     * @brief Command the UAV to land
     */
    void land();
    
    /**
     * @brief Execute emergency stop
     */
    void emergencyStop();
    
    /**
     * @brief Go to a position
     * @param position Target position
     */
    void goToPosition(const Eigen::Vector3d& position);
    
    /**
     * @brief Set velocity
     * @param velocity Target velocity
     */
    void setVelocity(const Eigen::Vector3d& velocity);
    
    
private:
    // Thread timing constants
    const double VISION_FREQ_HZ = 50.0;   // 50Hz for vision
    const double EKF_FREQ_HZ = 100.0;     // 100Hz for EKF
    const double CONTROL_FREQ_HZ = 50.0;  // 50Hz for control
    // Thread synchronization
    std::atomic<bool> running;
    std::mutex controllerMutex;
    
    // Thread handles
    std::thread visionThread;
    std::thread ekfThread;
    std::thread controlThread;
    std::thread displayThread;
    
    // Thread data exchange
    LatestData<ArucoPoseResult> arucoData;
    LatestData<EKFStateResult> stateData;
    LatestData<DroneControlOutput> controlData;
    
    // System components
    ImageSourcePtr imageSource;  // Use your existing ImageSourcePtr
    // Create communication module
    std::unique_ptr<MavlinkCommModule> commModule;
    std::string serialDevice = "/dev/ttyUSB0";
    int serialBaud = 57600;
    double frequency = 10.0;
    uint8_t systemId = 255;
    uint8_t componentId = 1;
    uint8_t targetSystemId = 1;
    uint8_t targetComponentId = 0;
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
    void displayThreadFunction();

    void sendControlToMAVLink(const DroneControlOutput& control);
    
    // Helper functions
    void createLogDirectory();
    void setupArucoPipeline();
    void setupEKFEstimator();
};

#endif // UAV_CONTROLLER_H