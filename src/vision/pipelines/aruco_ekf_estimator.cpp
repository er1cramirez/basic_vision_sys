#include "aruco_ekf_estimator.h"
#include <iostream>
#include <cmath>

// Initialize EKFStateResult with defaults
EKFStateResult::EKFStateResult()
    : position(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()),
      velocity(Eigen::Vector3d::Zero()),
      angularVelocity(Eigen::Vector3d::Zero()),
      acceleration(Eigen::Vector3d::Zero()),
      angularAcceleration(Eigen::Vector3d::Zero()),
      positionStdDev(Eigen::Vector3d::Zero()),
      velocityStdDev(Eigen::Vector3d::Zero()),
      accelerationStdDev(Eigen::Vector3d::Zero()),
      valid(false) {
    timestamp = std::chrono::steady_clock::now();
}

// Initialize EKFEstimatorConfig with defaults
EKFEstimatorConfig::EKFEstimatorConfig()
    : positionProcessNoise(0.01),         // 0.01 m²
      orientationProcessNoise(0.01),      // 0.01 quaternion units²
      velocityProcessNoise(0.1),          // 0.1 m²/s²
      angVelProcessNoise(0.1),            // 0.1 rad²/s²
      accelerationProcessNoise(0.5),      // 0.5 m²/s⁴
      angAccProcessNoise(0.5),            // 0.5 rad²/s⁴
      positionMeasurementNoise(0.01),     // 0.01 m²
      orientationMeasurementNoise(0.01),  // 0.01 quaternion units²
      predictionFrequencyHz(100.0),       // 100 Hz
      maxTimeDelta(0.1) {                 // 100 ms
}

// Constructor
ArucoEKFEstimator::ArucoEKFEstimator(const EKFEstimatorConfig& config)
    : config(config), initialized(false) {
    
    // Initialize state vector (19 elements)
    // [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, ax, ay, az, alpha_x, alpha_y, alpha_z]
    state = Eigen::VectorXd::Zero(19);
    
    // Initialize quaternion to identity rotation
    state(3) = 1.0;  // qw component
    
    // Initialize covariance matrix with large uncertainty
    covariance = Eigen::MatrixXd::Identity(19, 19);
    covariance.block<3,3>(0,0) *= 10.0;     // Position uncertainty
    covariance.block<4,4>(3,3) *= 10.0;     // Orientation uncertainty
    covariance.block<3,3>(7,7) *= 100.0;    // Velocity uncertainty
    covariance.block<3,3>(10,10) *= 100.0;  // Angular velocity uncertainty
    covariance.block<3,3>(13,13) *= 100.0;  // Acceleration uncertainty
    covariance.block<3,3>(16,16) *= 100.0;  // Angular acceleration uncertainty
    
    // Initialize noise covariance matrices
    updateProcessNoiseCovariance();
    updateMeasurementNoiseCovariance();
    
    // Initialize timestamps
    lastPredictionTime = std::chrono::steady_clock::now();
    lastMeasurementTime = lastPredictionTime;
}

// Update process noise covariance based on configuration
void ArucoEKFEstimator::updateProcessNoiseCovariance() {
    processNoiseCovariance = Eigen::MatrixXd::Identity(19, 19);
    processNoiseCovariance.block<3,3>(0,0) *= config.positionProcessNoise;      // Position
    processNoiseCovariance.block<4,4>(3,3) *= config.orientationProcessNoise;   // Orientation
    processNoiseCovariance.block<3,3>(7,7) *= config.velocityProcessNoise;      // Velocity
    processNoiseCovariance.block<3,3>(10,10) *= config.angVelProcessNoise;      // Angular velocity
    processNoiseCovariance.block<3,3>(13,13) *= config.accelerationProcessNoise; // Acceleration
    processNoiseCovariance.block<3,3>(16,16) *= config.angAccProcessNoise;      // Angular acceleration
}

// Update measurement noise covariance based on configuration
void ArucoEKFEstimator::updateMeasurementNoiseCovariance() {
    measurementNoiseCovariance = Eigen::MatrixXd::Identity(7, 7);
    measurementNoiseCovariance.block<3,3>(0,0) *= config.positionMeasurementNoise;      // Position
    measurementNoiseCovariance.block<4,4>(3,3) *= config.orientationMeasurementNoise;   // Orientation
}

// Initialize the estimator with an ArUco measurement
bool ArucoEKFEstimator::initialize(const ArucoPoseResult& measurement, 
                                 const std::chrono::time_point<std::chrono::steady_clock>& timestamp) {
    if (!measurement.detectionValid) {
        return false;
    }
    
    // Initialize position from measurement
    state.segment<3>(0) = measurement.position;
    
    // Initialize orientation from measurement
    state(3) = measurement.orientation.w();
    state(4) = measurement.orientation.x();
    state(5) = measurement.orientation.y();
    state(6) = measurement.orientation.z();
    
    // Initialize velocities to zero
    state.segment<3>(7).setZero();    // Linear velocity
    state.segment<3>(10).setZero();   // Angular velocity
    
    // Set initial uncertainty
    covariance = Eigen::MatrixXd::Identity(19, 19);
    covariance.block<3,3>(0,0) *= 0.01;     // Low position uncertainty
    covariance.block<4,4>(3,3) *= 0.01;     // Low orientation uncertainty 
    covariance.block<3,3>(7,7) *= 10.0;     // High velocity uncertainty
    covariance.block<3,3>(10,10) *= 10.0;   // High angular velocity uncertainty
    
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
    state = Eigen::VectorXd::Zero(19);
    state(3) = 1.0;  // qw component (identity quaternion)
    
    // Reset covariance
    covariance = Eigen::MatrixXd::Identity(19, 19);
    covariance.block<3,3>(0,0) *= 10.0;     // Position uncertainty
    covariance.block<4,4>(3,3) *= 10.0;     // Orientation uncertainty
    covariance.block<3,3>(7,7) *= 100.0;    // Velocity uncertainty
    covariance.block<3,3>(10,10) *= 100.0;  // Angular velocity uncertainty
    covariance.block<3,3>(13,13) *= 100.0;  // Acceleration uncertainty
    covariance.block<3,3>(16,16) *= 100.0;  // Angular acceleration uncertainty
    
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

// Normalize quaternion part of state vector
void ArucoEKFEstimator::normalizeQuaternion() {
    double norm = std::sqrt(state(3)*state(3) + state(4)*state(4) + 
                           state(5)*state(5) + state(6)*state(6));
    
    if (norm > 1e-10) {
        state.segment<4>(3) /= norm;
    } else {
        // Reset to identity if quaternion is too small
        state(3) = 1.0;
        state(4) = 0.0;
        state(5) = 0.0;
        state(6) = 0.0;
    }
}

Eigen::MatrixXd ArucoEKFEstimator::computeStateTransitionMatrix(double dt) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(19, 19);
    
    // Position updated by velocity and acceleration:
    // x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt^2
    F.block<3,3>(0,7) = Eigen::Matrix3d::Identity() * dt;          // Velocity contribution
    F.block<3,3>(0,13) = Eigen::Matrix3d::Identity() * 0.5 * dt*dt; // Acceleration contribution
    
    // Velocity updated by acceleration:
    // v(t+dt) = v(t) + a(t)*dt
    F.block<3,3>(7,13) = Eigen::Matrix3d::Identity() * dt;         // Acceleration contribution
    // Angular velocity updated by angular acceleration:
    F.block<3,3>(10,16) = Eigen::Matrix3d::Identity() * dt;
    // For orientation update, we'd ideally incorporate angular acceleration
    // This would require more complex quaternion kinematics
    
    return F;
}

// Compute process noise matrix for given time delta
Eigen::MatrixXd ArucoEKFEstimator::computeProcessNoiseMatrix(double dt) {
    // Scale process noise by dt for continuous-to-discrete conversion
    return processNoiseCovariance * dt;
}

// Compute measurement matrix (maps state to measurement)
Eigen::MatrixXd ArucoEKFEstimator::computeMeasurementMatrix() {
    // Measurement is [x, y, z, qw, qx, qy, qz]
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 19);
    
    // Position measurement directly observes position states
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    
    // Orientation measurement directly observes orientation states
    H.block<4,4>(3,3) = Eigen::Matrix4d::Identity();
    return H;
}

// Run prediction step
// Debug version of predict function with matrix dimension checks
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
    
    // 1. Predict state (this doesn't involve matrix multiplication, so it should be safe)
    // Position update: pos(t+dt) = pos(t) + vel(t)*dt + 0.5*acc(t)*dt^2
    state.segment<3>(0) += state.segment<3>(7) * dt + 0.5 * state.segment<3>(13) * dt * dt;
    
    // Velocity update: vel(t+dt) = vel(t) + acc(t)*dt
    state.segment<3>(7) += state.segment<3>(13) * dt;
    
    // Orientation update using quaternion kinematics
    Eigen::Vector3d omega = state.segment<3>(10);
    Eigen::Quaterniond q(state(3), state(4), state(5), state(6));
    
    if (omega.norm() > 1e-10) {
        // Compute quaternion differential
        Eigen::Quaterniond omega_quat(0, omega.x(), omega.y(), omega.z());
        Eigen::Quaterniond q_dot = q * omega_quat;
        
        // Scale the quaternion differential by 0.5
        q_dot.coeffs() *= 0.5;
        
        // Integrate quaternion: q(t+dt) = q(t) + q_dot*dt
        q.w() += q_dot.w() * dt;
        q.x() += q_dot.x() * dt;
        q.y() += q_dot.y() * dt;
        q.z() += q_dot.z() * dt;
        
        // Normalize
        q.normalize();
        
        // Update state
        state(3) = q.w();
        state(4) = q.x();
        state(5) = q.y();
        state(6) = q.z();
    }
    
    // Angular velocity update: omega(t+dt) = omega(t) + alpha(t)*dt
    state.segment<3>(10) += state.segment<3>(16) * dt;
    
    // Accelerations remain constant in the constant acceleration model
    
    // Normalize quaternion part of state vector
    normalizeQuaternion();
    
    // 2. Predict covariance - THIS IS WHERE THE ERROR LIKELY OCCURS
    
    // Try the matrix multiplication step by step
    try {
        Eigen::MatrixXd F_cov = F * covariance;
        
        Eigen::MatrixXd F_cov_Ft = F_cov * F.transpose();\
        
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

    
    // Create measurement vector
    Eigen::VectorXd z(7);
    z.segment<3>(0) = measurement.position;
    z(3) = measurement.orientation.w();
    z(4) = measurement.orientation.x();
    z(5) = measurement.orientation.y();
    z(6) = measurement.orientation.z();
    
    // Make sure quaternion is in the same hemisphere as predicted quaternion
    if (z(3)*state(3) + z(4)*state(4) + z(5)*state(5) + z(6)*state(6) < 0) {
        z.segment<4>(3) = -z.segment<4>(3); // Flip quaternion
    }
    
    // Get measurement matrix
    Eigen::MatrixXd H = computeMeasurementMatrix();
    
    // Calculate predicted measurement
    Eigen::VectorXd z_pred(7);
    z_pred.segment<3>(0) = state.segment<3>(0);  // Predicted position
    z_pred.segment<4>(3) = state.segment<4>(3);  // Predicted orientation
    
    // Calculate innovation (measurement residual)
    Eigen::VectorXd y = z - z_pred;
    
    // Calculate innovation covariance
    Eigen::MatrixXd S = H * covariance * H.transpose() + measurementNoiseCovariance;
    
    // Calculate Kalman gain
    Eigen::MatrixXd K = covariance * H.transpose() * S.inverse();
    
    // Update state
    state = state + K * y;
    
    // Normalize quaternion
    normalizeQuaternion();
    
    // Update covariance - FIXED: Use 19x19 identity matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(19, 19);  // ✅ Changed from 13,13
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
    result.orientation = Eigen::Quaterniond(state(3), state(4), state(5), state(6));
    result.velocity = state.segment<3>(7);
    result.angularVelocity = state.segment<3>(10);
    result.acceleration = state.segment<3>(13);
    result.angularAcceleration = state.segment<3>(16);
    
    // Extract standard deviations from covariance diagonal
    for (int i = 0; i < 3; i++) {
        result.positionStdDev(i) = std::sqrt(covariance(i, i));
        result.velocityStdDev(i) = std::sqrt(covariance(i+7, i+7));
        result.accelerationStdDev(i) = std::sqrt(covariance(i+13, i+13));
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