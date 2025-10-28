// include/vision/core/GzOdometrySource.h
#ifndef GZ_ODOMETRY_SOURCE_H
#define GZ_ODOMETRY_SOURCE_H

#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>

/**
 * @class GzOdometrySource
 * @brief Subscribes to Gazebo odometry topics for drone and platform
 *        to provide ground truth data for EKF evaluation
 * 
 * This module:
 * - Subscribes to drone and platform odometry topics
 * - Calculates relative position and velocity
 * - Logs all data using the Logger singleton
 * - Runs independently without blocking the main controller
 */
class GzOdometrySource {
public:
    /**
     * @brief Configuration structure for odometry source
     */
    struct Config {
        std::string droneTopic;      // Gazebo topic for drone odometry
        std::string platformTopic;   // Gazebo topic for platform odometry
        double loggingRate;          // Logging rate in Hz (default: 50.0)
        bool logRawData;             // Log raw drone and platform data
        bool logRelativeData;        // Log relative position/velocity
        
        Config() 
            : droneTopic("/model/iris_with_fixed_camera/odometry"),
              platformTopic("/model/moving_platform/odometry"),
              loggingRate(60.0),
              logRawData(true),
              logRelativeData(true) {}
    };
    
    /**
     * @brief Constructor with configuration
     * @param config Configuration parameters
     */
    explicit GzOdometrySource(const Config& config = Config());
    
    /**
     * @brief Destructor
     */
    ~GzOdometrySource();
    
    /**
     * @brief Initialize and subscribe to Gazebo topics
     * @return true if successful
     */
    bool initialize();
    
    /**
     * @brief Start the logging thread
     * @return true if successful
     */
    bool start();
    
    /**
     * @brief Stop the logging thread
     */
    void stop();
    
    /**
     * @brief Check if initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized; }
    
    /**
     * @brief Check if running
     * @return true if running
     */
    bool isRunning() const { return running; }
    
    /**
     * @brief Get current relative position
     * @param dx Output: relative x position
     * @param dy Output: relative y position
     * @param dz Output: relative z position
     * @return true if data is available
     */
    bool getRelativePosition(double& dx, double& dy, double& dz) const;
    
    /**
     * @brief Get current relative velocity
     * @param dvx Output: relative x velocity
     * @param dvy Output: relative y velocity
     * @param dvz Output: relative z velocity
     * @return true if data is available
     */
    bool getRelativeVelocity(double& dvx, double& dvy, double& dvz) const;
    
private:
    /**
     * @brief Callback for drone odometry messages
     */
    void onDroneOdometry(const gz::msgs::Odometry& msg);
    
    /**
     * @brief Callback for platform odometry messages
     */
    void onPlatformOdometry(const gz::msgs::Odometry& msg);
    
    /**
     * @brief Logging thread function
     */
    void loggingThread();
    
    /**
     * @brief Calculate relative position and velocity
     */
    void calculateRelative();
    
    /**
     * @brief Transform velocity from body frame to world frame using quaternion
     * @param vx_body, vy_body, vz_body: velocity in body frame
     * @param qw, qx, qy, qz: orientation quaternion
     * @param vx_world, vy_world, vz_world: output velocity in world frame
     */
    void transformVelocityToWorld(double vx_body, double vy_body, double vz_body,
                                   double qw, double qx, double qy, double qz,
                                   double& vx_world, double& vy_world, double& vz_world);
    
    // Configuration
    Config config;
    
    // Gazebo transport
    gz::transport::Node node;
    
    // State flags
    std::atomic<bool> initialized;
    std::atomic<bool> running;
    std::atomic<bool> hasDroneData;
    std::atomic<bool> hasPlatformData;
    
    // Drone state
    struct OdometryData {
        double x, y, z;           // Position (world frame)
        double qw, qx, qy, qz;    // Orientation quaternion
        double vx, vy, vz;        // Velocity (world frame - after transformation)
        double vx_local, vy_local, vz_local;  // Velocity (body frame - raw from Gazebo)
        double wx, wy, wz;        // Angular velocity (body frame)
        uint64_t timestamp;       // Microseconds
        bool valid;
        
        OdometryData() : x(0), y(0), z(0), 
                        qw(1), qx(0), qy(0), qz(0),
                        vx(0), vy(0), vz(0), 
                        vx_local(0), vy_local(0), vz_local(0),
                        wx(0), wy(0), wz(0),
                        timestamp(0), valid(false) {}
    };
    
    OdometryData droneData;
    OdometryData platformData;
    
    // Relative state
    struct RelativeData {
        double dx, dy, dz;      // Relative position
        double dvx, dvy, dvz;   // Relative velocity
        bool valid;
        
        RelativeData() : dx(0), dy(0), dz(0), dvx(0), dvy(0), dvz(0), 
                        valid(false) {}
    };
    
    RelativeData relativeData;
    
    // Thread synchronization
    mutable std::mutex dataMutex;
    std::thread loggingThreadHandle;
    
    // Timing
    std::chrono::steady_clock::time_point startTime;
};

#endif // GZ_ODOMETRY_SOURCE_H