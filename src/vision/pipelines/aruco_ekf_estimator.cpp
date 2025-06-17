#include "aruco_ekf_estimator.h"
#include <iostream>
#include <cmath>

// Initialize EKFStateResult with defaults
EKFStateResult::EKFStateResult()
    : position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      acceleration(Eigen::Vector3d::Zero()),
      positionStdDev(Eigen::Vector3d::Zero()),
      velocityStdDev(Eigen::Vector3d::Zero()),
      accelerationStdDev(Eigen::Vector3d::Zero()),
      valid(false) {
    timestamp = std::chrono::steady_clock::now();
}

// Initialize EKFEstimatorConfig with defaults
EKFEstimatorConfig::EKFEstimatorConfig()
    : positionProcessNoise(0.01),         // 0.01 m²
      velocityProcessNoise(0.1),          // 0.1 m²/s²
      accelerationProcessNoise(0.5),      // 0.5 m²/s⁴
      positionMeasurementNoise(0.01),     // 0.01 m²
      predictionFrequencyHz(50.0),       // 50 Hz
      maxTimeDelta(0.1) {                 // 100 ms
}

// Constructor
ArucoEKFEstimator::ArucoEKFEstimator(const EKFEstimatorConfig& config)
    : config(config), initialized(false) {
    
    // Initialize state vector (9 elements)
    // [x, y, z, vx, vy, vz, ax, ay, az]
    state = Eigen::VectorXd::Zero(9);
    
    // Initialize covariance matrix with large uncertainty
    covariance = Eigen::MatrixXd::Identity(9, 9);
    covariance.block<3,3>(0,0) *= 10.0;     // Position uncertainty
    covariance.block<3,3>(3,3) *= 100.0;    // Velocity uncertainty
    covariance.block<3,3>(6,6) *= 100.0;    // Acceleration uncertainty
    
    // Initialize noise covariance matrices
    updateProcessNoiseCovariance();
    updateMeasurementNoiseCovariance();
    
    // Initialize timestamps
    lastPredictionTime = std::chrono::steady_clock::now();
    lastMeasurementTime = lastPredictionTime;
}

// Update process noise covariance based on configuration
void ArucoEKFEstimator::updateProcessNoiseCovariance() {
    processNoiseCovariance = Eigen::MatrixXd::Identity(9, 9);
    processNoiseCovariance.block<3,3>(0,0) *= config.positionProcessNoise;      // Position
    processNoiseCovariance.block<3,3>(3,3) *= config.velocityProcessNoise;      // Velocity
    processNoiseCovariance.block<3,3>(6,6) *= config.accelerationProcessNoise;  // Acceleration
}

// Update measurement noise covariance based on configuration
void ArucoEKFEstimator::updateMeasurementNoiseCovariance() {
    measurementNoiseCovariance = Eigen::MatrixXd::Identity(3, 3);
    measurementNoiseCovariance.block<3,3>(0,0) *= config.positionMeasurementNoise;      // Position
}

// Initialize the estimator with an ArUco measurement
bool ArucoEKFEstimator::initialize(const ArucoPoseResult& measurement, 
                                 const std::chrono::time_point<std::chrono::steady_clock>& timestamp) {
    if (!measurement.detectionValid) {
        return false;
    }
    
    // Initialize position from measurement
    state.segment<3>(0) = measurement.position;
    
    // Initialize velocities and accelerations to zero
    state.segment<3>(3).setZero();    // Linear velocity
    state.segment<3>(6).setZero();    // Linear acceleration
    
    // Set initial uncertainty
    covariance = Eigen::MatrixXd::Identity(9, 9);
    covariance.block<3,3>(0,0) *= 0.01;     // Low position uncertainty
    covariance.block<3,3>(3,3) *= 10.0;     // High velocity uncertainty
    covariance.block<3,3>(6,6) *= 10.0;     // High acceleration uncertainty
    
    // Update timestamps
    lastPredictionTime = timestamp;
    lastMeasurementTime = timestamp;
    
    // Mark as initialized
    initialized = true;
    
    return true;
}

// Reset the estimator
void ArucoEKFEstimator::reset() {
    // Reset state
    state = Eigen::VectorXd::Zero(9);
    
    // Reset covariance
    covariance = Eigen::MatrixXd::Identity(9, 9);
    covariance.block<3,3>(0,0) *= 10.0;     // Position uncertainty
    covariance.block<3,3>(3,3) *= 100.0;    // Velocity uncertainty
    covariance.block<3,3>(6,6) *= 100.0;    // Acceleration uncertainty
    
    // Reset flags
    initialized = false;
    
    // Reset timestamps
    lastPredictionTime = std::chrono::steady_clock::now();
    lastMeasurementTime = lastPredictionTime;
}

// Update configuration
void ArucoEKFEstimator::updateConfig(const EKFEstimatorConfig& newConfig) {
    config = newConfig;
    updateProcessNoiseCovariance();
    updateMeasurementNoiseCovariance();
}

// Get configuration
const EKFEstimatorConfig& ArucoEKFEstimator::getConfig() const {
    return config;
}

Eigen::MatrixXd ArucoEKFEstimator::computeStateTransitionMatrix(double dt) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    
    // Position updated by velocity and acceleration:
    // x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
    F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;          // Velocity contribution
    F.block<3,3>(0,6) = Eigen::Matrix3d::Identity() * 0.5 * dt*dt; // Acceleration contribution
    
    // Velocity updated by acceleration:
    // v(t+dt) = v(t) + a(t)*dt
    F.block<3,3>(3,6) = Eigen::Matrix3d::Identity() * dt;         // Acceleration contribution
    
    return F;
}

// Compute process noise matrix for given time delta
Eigen::MatrixXd ArucoEKFEstimator::computeProcessNoiseMatrix(double dt) {
    // Scale process noise by dt for continuous-to-discrete conversion
    return processNoiseCovariance * dt;
}

// Compute measurement matrix (maps state to measurement)
Eigen::MatrixXd ArucoEKFEstimator::computeMeasurementMatrix() {
    // Measurement is [x, y, z] (position only)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 9);
    
    // Position measurement directly observes position states
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    
    return H;
}

// Run prediction step
bool ArucoEKFEstimator::predict(const std::chrono::time_point<std::chrono::steady_clock>& timestamp) {
    if (!initialized) {
        return false;
    }
    
    // Calculate time delta
    double dt = std::chrono::duration<double>(timestamp - lastPredictionTime).count();
    
    // Skip prediction if dt is too small or too large
    if (dt < 1e-6 || dt > config.maxTimeDelta) {
        if (dt > config.maxTimeDelta) {
            std::cerr << "Warning: Time delta too large in EKF predict: " << dt << " s" << std::endl;
        }
        lastPredictionTime = timestamp;
        return false;
    }
    
    // Get state transition matrix
    Eigen::MatrixXd F = computeStateTransitionMatrix(dt);
    
    // Get process noise matrix
    Eigen::MatrixXd Q = computeProcessNoiseMatrix(dt);
    
    // 1. Predict state
    // Position update: pos(t+dt) = pos(t) + vel(t)*dt + 0.5*acc(t)*dt^2
    state.segment<3>(0) += state.segment<3>(3) * dt + 0.5 * state.segment<3>(6) * dt * dt;
    
    // Velocity update: vel(t+dt) = vel(t) + acc(t)*dt
    state.segment<3>(3) += state.segment<3>(6) * dt;
    
    // Accelerations remain constant in the constant acceleration model
    
    // 2. Predict covariance
    try {
        Eigen::MatrixXd F_cov = F * covariance;
        Eigen::MatrixXd F_cov_Ft = F_cov * F.transpose();
        covariance = F_cov_Ft + Q;
    } catch (const std::exception& e) {
        std::cerr << "Exception in covariance update: " << e.what() << std::endl;
        return false;
    }
    
    // Update timestamp
    lastPredictionTime = timestamp;
    
    return true;
}

// Process a new measurement
bool ArucoEKFEstimator::update(const ArucoPoseResult& measurement,
                            const std::chrono::time_point<std::chrono::steady_clock>& timestamp) {
    // If not initialized, try to initialize with this measurement
    if (!initialized) {
        return initialize(measurement, timestamp);
    }
    
    // Skip invalid measurements
    if (!measurement.detectionValid) {
        return false;
    }
    
    // Run prediction up to measurement time
    predict(timestamp);
    
    // Create measurement vector (position only)
    Eigen::VectorXd z(3);
    z.segment<3>(0) = measurement.position;
    
    // Get measurement matrix
    Eigen::MatrixXd H = computeMeasurementMatrix();
    
    // Calculate predicted measurement
    Eigen::VectorXd z_pred(3);
    z_pred.segment<3>(0) = state.segment<3>(0);  // Predicted position
    
    // Calculate innovation (measurement residual)
    Eigen::VectorXd y = z - z_pred;
    
    // Calculate innovation covariance
    Eigen::MatrixXd S = H * covariance * H.transpose() + measurementNoiseCovariance;
    
    // Calculate Kalman gain
    Eigen::MatrixXd K = covariance * H.transpose() * S.inverse();
    
    // Update state
    state = state + K * y;
    
    // Update covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
    covariance = (I - K * H) * covariance;
    
    // Ensure covariance stays symmetric and positive definite
    covariance = (covariance + covariance.transpose()) * 0.5;
    
    // Update timestamp
    lastMeasurementTime = timestamp;
    
    return true;
}

// Convert state vector to result structure
EKFStateResult ArucoEKFEstimator::stateVectorToResult() const {
    EKFStateResult result;
    
    result.timestamp = lastPredictionTime;
    result.valid = initialized;
    
    // Extract state components
    result.position = state.segment<3>(0);
    result.velocity = state.segment<3>(3);
    result.acceleration = state.segment<3>(6);
    
    // Extract standard deviations from covariance diagonal
    for (int i = 0; i < 3; i++) {
        result.positionStdDev(i) = std::sqrt(covariance(i, i));
        result.velocityStdDev(i) = std::sqrt(covariance(i+3, i+3));
        result.accelerationStdDev(i) = std::sqrt(covariance(i+6, i+6));
    }
    
    return result;
}

// Get latest state estimate
EKFStateResult ArucoEKFEstimator::getStateEstimate() const {
    return stateVectorToResult();
}

// Check if initialized
bool ArucoEKFEstimator::isInitialized() const {
    return initialized;
}

// Get last prediction time
std::chrono::time_point<std::chrono::steady_clock> ArucoEKFEstimator::getLastPredictionTime() const {
    return lastPredictionTime;
}

// Get last measurement time
std::chrono::time_point<std::chrono::steady_clock> ArucoEKFEstimator::getLastMeasurementTime() const {
    return lastMeasurementTime;
}