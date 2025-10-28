// src/vision/core/GzOdometrySource.cpp
#include "GzOdometrySource.h"
#include "Logger.h"
#include <iostream>
#include <cmath>

GzOdometrySource::GzOdometrySource(const Config& config)
    : config(config),
      initialized(false),
      running(false),
      hasDroneData(false),
      hasPlatformData(false) {
    startTime = std::chrono::steady_clock::now();
}

GzOdometrySource::~GzOdometrySource() {
    stop();
}

bool GzOdometrySource::initialize() {
    if (initialized) {
        std::cout << "GzOdometrySource: Already initialized" << std::endl;
        return true;
    }
    
    std::cout << "GzOdometrySource: Initializing..." << std::endl;
    std::cout << "  Drone topic: " << config.droneTopic << std::endl;
    std::cout << "  Platform topic: " << config.platformTopic << std::endl;
    std::cout << "  Logging rate: " << config.loggingRate << " Hz" << std::endl;
    
    // Subscribe to drone odometry topic
    if (!node.Subscribe(config.droneTopic, &GzOdometrySource::onDroneOdometry, this)) {
        std::cerr << "GzOdometrySource: Failed to subscribe to drone topic: " 
                  << config.droneTopic << std::endl;
        return false;
    }
    std::cout << "  Subscribed to drone odometry" << std::endl;
    
    // Subscribe to platform odometry topic
    if (!node.Subscribe(config.platformTopic, &GzOdometrySource::onPlatformOdometry, this)) {
        std::cerr << "GzOdometrySource: Failed to subscribe to platform topic: " 
                  << config.platformTopic << std::endl;
        return false;
    }
    std::cout << "  Subscribed to platform odometry" << std::endl;
    
    initialized = true;
    std::cout << "GzOdometrySource: Initialized successfully" << std::endl;
    
    return true;
}

bool GzOdometrySource::start() {
    if (!initialized) {
        std::cerr << "GzOdometrySource: Cannot start - not initialized" << std::endl;
        return false;
    }
    
    if (running) {
        std::cout << "GzOdometrySource: Already running" << std::endl;
        return true;
    }
    
    // Check if Logger is initialized
    if (!Logger::getInstance().isInitialized()) {
        std::cerr << "GzOdometrySource: Logger not initialized" << std::endl;
        return false;
    }
    
    std::cout << "GzOdometrySource: Starting logging thread..." << std::endl;
    
    running = true;
    loggingThreadHandle = std::thread(&GzOdometrySource::loggingThread, this);
    
    std::cout << "GzOdometrySource: Logging thread started" << std::endl;
    
    return true;
}

void GzOdometrySource::stop() {
    if (running) {
        std::cout << "GzOdometrySource: Stopping..." << std::endl;
        running = false;
        
        if (loggingThreadHandle.joinable()) {
            loggingThreadHandle.join();
        }
        
        std::cout << "GzOdometrySource: Stopped" << std::endl;
    }
    
    if (initialized) {
        // Unsubscribe from topics
        node.Unsubscribe(config.droneTopic);
        node.Unsubscribe(config.platformTopic);
        initialized = false;
    }
}

void GzOdometrySource::onDroneOdometry(const gz::msgs::Odometry& msg) {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    // Extract position (world frame)
    if (msg.has_pose() && msg.pose().has_position()) {
        droneData.x = msg.pose().position().x();
        droneData.y = msg.pose().position().y();
        droneData.z = msg.pose().position().z();
    }
    
    // Extract orientation (quaternion)
    if (msg.has_pose() && msg.pose().has_orientation()) {
        droneData.qw = msg.pose().orientation().w();
        droneData.qx = msg.pose().orientation().x();
        droneData.qy = msg.pose().orientation().y();
        droneData.qz = msg.pose().orientation().z();
    }
    
    // Extract velocity (body frame) and angular velocity
    if (msg.has_twist()) {
        if (msg.twist().has_linear()) {
            droneData.vx_local = msg.twist().linear().x();
            droneData.vy_local = msg.twist().linear().y();
            droneData.vz_local = msg.twist().linear().z();
            
            // Transform velocity from body frame to world frame
            transformVelocityToWorld(
                droneData.vx_local, droneData.vy_local, droneData.vz_local,
                droneData.qw, droneData.qx, droneData.qy, droneData.qz,
                droneData.vx, droneData.vy, droneData.vz
            );
        }
        
        if (msg.twist().has_angular()) {
            droneData.wx = msg.twist().angular().x();
            droneData.wy = msg.twist().angular().y();
            droneData.wz = msg.twist().angular().z();
        }
    }
    
    // Timestamp
    auto now = std::chrono::steady_clock::now();
    droneData.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        now - startTime).count();
    
    droneData.valid = true;
    hasDroneData = true;
}

void GzOdometrySource::onPlatformOdometry(const gz::msgs::Odometry& msg) {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    // Extract position (world frame)
    if (msg.has_pose() && msg.pose().has_position()) {
        platformData.x = msg.pose().position().x();
        platformData.y = msg.pose().position().y();
        platformData.z = msg.pose().position().z();
    }
    
    // Extract orientation (quaternion)
    if (msg.has_pose() && msg.pose().has_orientation()) {
        platformData.qw = msg.pose().orientation().w();
        platformData.qx = msg.pose().orientation().x();
        platformData.qy = msg.pose().orientation().y();
        platformData.qz = msg.pose().orientation().z();
    }
    
    // Extract velocity (body frame) and angular velocity
    if (msg.has_twist()) {
        if (msg.twist().has_linear()) {
            platformData.vx_local = msg.twist().linear().x();
            platformData.vy_local = msg.twist().linear().y();
            platformData.vz_local = msg.twist().linear().z();
            
            // Transform velocity from body frame to world frame
            transformVelocityToWorld(
                platformData.vx_local, platformData.vy_local, platformData.vz_local,
                platformData.qw, platformData.qx, platformData.qy, platformData.qz,
                platformData.vx, platformData.vy, platformData.vz
            );
        }
        
        if (msg.twist().has_angular()) {
            platformData.wx = msg.twist().angular().x();
            platformData.wy = msg.twist().angular().y();
            platformData.wz = msg.twist().angular().z();
        }
    }
    
    // Timestamp
    auto now = std::chrono::steady_clock::now();
    platformData.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        now - startTime).count();
    
    platformData.valid = true;
    hasPlatformData = true;
}

void GzOdometrySource::calculateRelative() {
    // This should be called with dataMutex locked
    
    if (!droneData.valid || !platformData.valid) {
        relativeData.valid = false;
        return;
    }
    
    // Calculate relative position: platform - drone
    // This gives the position of the platform as seen from the drone
    // (but expressed in world frame coordinates)
    relativeData.dx = droneData.x - platformData.x;
    relativeData.dy = droneData.y - platformData.y;
    relativeData.dz = droneData.z - platformData.z;

    // Calculate relative velocity: platform - drone
    // This gives the velocity of the platform relative to the drone
    // (expressed in world frame coordinates)
    relativeData.dvx = droneData.vx - platformData.vx;
    relativeData.dvy = droneData.vy - platformData.vy;
    relativeData.dvz = droneData.vz - platformData.vz;

    relativeData.valid = true;
}

void GzOdometrySource::transformVelocityToWorld(double vx_body, double vy_body, double vz_body,
                                                 double qw, double qx, double qy, double qz,
                                                 double& vx_world, double& vy_world, double& vz_world) {
    // Normalize quaternion (just in case)
    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }
    
    // Convert quaternion to rotation matrix
    // R transforms from body frame to world frame
    // Formula: v_world = R * v_body
    
    // Rotation matrix elements
    double r00 = 1.0 - 2.0*(qy*qy + qz*qz);
    double r01 = 2.0*(qx*qy - qw*qz);
    double r02 = 2.0*(qx*qz + qw*qy);
    
    double r10 = 2.0*(qx*qy + qw*qz);
    double r11 = 1.0 - 2.0*(qx*qx + qz*qz);
    double r12 = 2.0*(qy*qz - qw*qx);
    
    double r20 = 2.0*(qx*qz - qw*qy);
    double r21 = 2.0*(qy*qz + qw*qx);
    double r22 = 1.0 - 2.0*(qx*qx + qy*qy);
    
    // Apply rotation: v_world = R * v_body
    vx_world = r00*vx_body + r01*vy_body + r02*vz_body;
    vy_world = r10*vx_body + r11*vy_body + r12*vz_body;
    vz_world = r20*vx_body + r21*vy_body + r22*vz_body;
}

void GzOdometrySource::loggingThread() {
    std::cout << "GzOdometrySource: Logging thread running" << std::endl;
    
    // Calculate sleep duration based on logging rate
    auto sleepDuration = std::chrono::microseconds(
        static_cast<int64_t>(1000000.0 / config.loggingRate));
    
    auto nextLogTime = std::chrono::steady_clock::now();
    
    while (running) {
        // Wait until next log time
        std::this_thread::sleep_until(nextLogTime);
        nextLogTime += sleepDuration;
        
        // Get current timestamp
        uint64_t timestamp = Logger::getInstance().getMicroseconds();
        
        // Copy data with lock
        OdometryData droneSnapshot;
        OdometryData platformSnapshot;
        RelativeData relativeSnapshot;
        
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            droneSnapshot = droneData;
            platformSnapshot = platformData;
            
            // Calculate relative data
            calculateRelative();
            relativeSnapshot = relativeData;
        }
        
        // Log drone data (GTDP)
        if (config.logRawData && droneSnapshot.valid) {
            Logger::getInstance().Write(
                "GTDP",
                "timestamp_us,x,y,z,vx,vy,vz",
                "Qdddddd",
                timestamp,
                droneSnapshot.x, droneSnapshot.y, droneSnapshot.z,
                droneSnapshot.vx, droneSnapshot.vy, droneSnapshot.vz
            );
        }
        
        // Log platform data (GTPP)
        if (config.logRawData && platformSnapshot.valid) {
            Logger::getInstance().Write(
                "GTPP",
                "timestamp_us,x,y,z,vx,vy,vz",
                "Qdddddd",
                timestamp,
                platformSnapshot.x, platformSnapshot.y, platformSnapshot.z,
                platformSnapshot.vx, platformSnapshot.vy, platformSnapshot.vz
            );
        }
        
        // Log relative position (GTPR)
        if (config.logRelativeData && relativeSnapshot.valid) {
            Logger::getInstance().Write(
                "GTPR",
                "timestamp_us,dx,dy,dz",
                "Qddd",
                timestamp,
                relativeSnapshot.dx, relativeSnapshot.dy, relativeSnapshot.dz
            );
        }
        
        // Log relative velocity (GTVR)
        if (config.logRelativeData && relativeSnapshot.valid) {
            Logger::getInstance().Write(
                "GTVR",
                "timestamp_us,dvx,dvy,dvz",
                "Qddd",
                timestamp,
                relativeSnapshot.dvx, relativeSnapshot.dvy, relativeSnapshot.dvz
            );
        }
    }
    
    std::cout << "GzOdometrySource: Logging thread finished" << std::endl;
}

bool GzOdometrySource::getRelativePosition(double& dx, double& dy, double& dz) const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    if (!relativeData.valid) {
        return false;
    }
    
    dx = relativeData.dx;
    dy = relativeData.dy;
    dz = relativeData.dz;
    
    return true;
}

bool GzOdometrySource::getRelativeVelocity(double& dvx, double& dvy, double& dvz) const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    if (!relativeData.valid) {
        return false;
    }
    
    dvx = relativeData.dvx;
    dvy = relativeData.dvy;
    dvz = relativeData.dvz;
    
    return true;
}