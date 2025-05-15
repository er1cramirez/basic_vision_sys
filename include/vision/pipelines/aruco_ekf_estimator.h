#ifndef ARUCO_EKF_ESTIMATOR_H
#define ARUCO_EKF_ESTIMATOR_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <optional>
#include "aruco_pose_pipeline.h"

/**
 * @brief Result of EKF state estimation
 */
struct EKFStateResult {
    // Timing information
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
    
    // State components
    Eigen::Vector3d position;       // Position in reference frame
    Eigen::Quaterniond orientation; // Orientation in reference frame
    Eigen::Vector3d velocity;       // Linear velocity in reference frame
    Eigen::Vector3d angularVelocity; // Angular velocity in reference frame
    
    // Uncertainty metrics
    Eigen::Vector3d positionStdDev;  // Standard deviation of position estimate
    Eigen::Vector3d velocityStdDev;  // Standard deviation of velocity estimate
    
    // Validity flag
    bool valid;
    
    // Constructor with defaults
    EKFStateResult();
};

/**
 * @brief Configuration for the EKF state estimator
 */
struct EKFEstimatorConfig {
    // Process noise parameters (variances)
    double positionProcessNoise;      // Position process noise (m^2)
    double orientationProcessNoise;   // Orientation process noise (quaternion units^2)
    double velocityProcessNoise;      // Velocity process noise (m^2/s^2)
    double angVelProcessNoise;        // Angular velocity process noise (rad^2/s^2)
    
    // Measurement noise parameters (variances)
    double positionMeasurementNoise;  // Position measurement noise (m^2)
    double orientationMeasurementNoise; // Orientation measurement noise (quaternion units^2)
    
    // Filter parameters
    double predictionFrequencyHz;     // Frequency at which to run prediction steps
    double maxTimeDelta;              // Maximum time delta for prediction (s)
    
    // Constructor with default values
    EKFEstimatorConfig();
};

/**
 * @brief Extended Kalman Filter for ArUco pose state estimation
 */
class ArucoEKFEstimator {
public:
    /**
     * @brief Constructor
     * @param config Configuration parameters for the estimator
     */
    ArucoEKFEstimator(const EKFEstimatorConfig& config = EKFEstimatorConfig());
    
    /**
     * @brief Destructor
     */
    ~ArucoEKFEstimator() = default;
    
    /**
     * @brief Initialize the estimator with an initial measurement
     * @param measurement ArUco pose measurement to initialize from
     * @param timestamp Timestamp of the measurement
     * @return True if initialization was successful
     */
    bool initialize(const ArucoPoseResult& measurement, 
                   const std::chrono::time_point<std::chrono::steady_clock>& timestamp);
    
    /**
     * @brief Reset the estimator to initial state
     */
    void reset();
    
    /**
     * @brief Update the configuration
     * @param config New configuration
     */
    void updateConfig(const EKFEstimatorConfig& config);
    
    /**
     * @brief Get the current configuration
     * @return Current configuration
     */
    const EKFEstimatorConfig& getConfig() const;
    
    /**
     * @brief Process a new ArUco pose measurement
     * @param measurement ArUco pose result to process
     * @param timestamp Timestamp of the measurement
     * @return True if the update was successful
     */
    bool update(const ArucoPoseResult& measurement,
               const std::chrono::time_point<std::chrono::steady_clock>& timestamp);
    
    /**
     * @brief Run a prediction step to the specified time
     * @param timestamp Time to predict to
     * @return True if prediction was successful
     */
    bool predict(const std::chrono::time_point<std::chrono::steady_clock>& timestamp);
    
    /**
     * @brief Get the latest state estimate
     * @return State estimate result
     */
    EKFStateResult getStateEstimate() const;
    
    /**
     * @brief Check if the estimator has been initialized
     * @return True if initialized
     */
    bool isInitialized() const;
    
    /**
     * @brief Get the last prediction timestamp
     * @return Last prediction timestamp
     */
    std::chrono::time_point<std::chrono::steady_clock> getLastPredictionTime() const;
    
    /**
     * @brief Get the last measurement timestamp
     * @return Last measurement timestamp
     */
    std::chrono::time_point<std::chrono::steady_clock> getLastMeasurementTime() const;
    
private:
    // State vector: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
    // (position, orientation quaternion, linear velocity, angular velocity)
    Eigen::VectorXd state;
    
    // State covariance matrix
    Eigen::MatrixXd covariance;
    
    // Process noise covariance
    Eigen::MatrixXd processNoiseCovariance;
    
    // Measurement noise covariance
    Eigen::MatrixXd measurementNoiseCovariance;
    
    // Configuration
    EKFEstimatorConfig config;
    
    // Status flags
    bool initialized;
    
    // Timing
    std::chrono::time_point<std::chrono::steady_clock> lastPredictionTime;
    std::chrono::time_point<std::chrono::steady_clock> lastMeasurementTime;
    
    // Helper methods
    void updateProcessNoiseCovariance();
    void updateMeasurementNoiseCovariance();
    void normalizeQuaternion();
    
    // Matrix computation methods
    Eigen::MatrixXd computeStateTransitionMatrix(double dt);
    Eigen::MatrixXd computeProcessNoiseMatrix(double dt);
    Eigen::MatrixXd computeMeasurementMatrix();
    
    // Convert between state vector and result structure
    EKFStateResult stateVectorToResult() const;
};

#endif // ARUCO_EKF_ESTIMATOR_H