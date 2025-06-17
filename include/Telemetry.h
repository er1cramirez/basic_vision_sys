#ifndef TELEMETRY_H
#define TELEMETRY_H

/**
 * @file Telemetry.h
 * @brief Simple header-only wrapper for the generic telemetry system
 * 
 * This provides WPILib SmartDashboard-style convenience functions for logging telemetry data.
 * Just include this header and use the TEL_* macros or Telemetry:: functions.
 * 
 * Usage examples:
 *   TEL_NUMBER("control/position_x", currentPosition.x);
 *   TEL_VECTOR("control/velocity", vel.x, vel.y, vel.z);
 *   TEL_EVENT("System initialized");
 *   TEL_TIMER_START("control_loop");
 *   // ... do work ...
 *   TEL_TIMER_END("control_loop");
 */

#include "telemetry/TelemetryPublisher.h"

// Convenience macros for quick telemetry logging
#define TEL_NUMBER(name, value) Telemetry::putNumber(name, value)
#define TEL_VECTOR(name, x, y, z) Telemetry::putVector3(name, x, y, z)
#define TEL_BOOL(name, value) Telemetry::putBoolean(name, value)
#define TEL_STRING(name, value) Telemetry::putString(name, value)
#define TEL_ARRAY(name, array) Telemetry::putNumberArray(name, array)

#define TEL_EVENT(message) Telemetry::logEvent(message)
#define TEL_INFO(message) Telemetry::logInfo(message)
#define TEL_WARNING(message) Telemetry::logWarning(message)
#define TEL_ERROR(message) Telemetry::logError(message)

#define TEL_TIMER_START(name) Telemetry::startTimer(name)
#define TEL_TIMER_END(name) Telemetry::endTimer(name)

#define TEL_FPS(component, fps) Telemetry::recordFPS(component, fps)
#define TEL_FRAME_TIME(component, time_ms) Telemetry::recordFrameTime(component, time_ms)

// RAII Timer class for automatic timing
class TelemetryTimer {
private:
    std::string timerName;
    std::chrono::steady_clock::time_point startTime;
    bool ended;
    
public:
    explicit TelemetryTimer(const std::string& name) 
        : timerName(name), startTime(std::chrono::steady_clock::now()), ended(false) {
        // Don't use the global timer functions since we're handling it ourselves
    }
    
    ~TelemetryTimer() {
        if (!ended) {
            end();
        }
    }
    
    void end() {
        if (!ended) {
            auto duration = std::chrono::steady_clock::now() - startTime;
            auto durationMs = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
            Telemetry::putNumber("timers/" + timerName, durationMs);
            ended = true;
        }
    }
    
    double getElapsedMs() const {
        auto duration = std::chrono::steady_clock::now() - startTime;
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
    }
};

// Macro for RAII timing
#define TEL_AUTO_TIMER(name) TelemetryTimer _tel_timer_##__LINE__(name)

// Function-scoped timer
#define TEL_FUNCTION_TIMER() TEL_AUTO_TIMER(__FUNCTION__)

// Telemetry initialization helper
class TelemetryInit {
private:
    static bool initialized;
    
public:
    static bool initialize(uint16_t udpPort = 14559, uint32_t rateHz = 20) {
        if (!initialized) {
            auto publisher = Telemetry::getPublisher();
            initialized = publisher->initialize(udpPort, rateHz);
            if (initialized) {
                TEL_INFO("Telemetry system initialized on port " + std::to_string(udpPort));
            }
        }
        return initialized;
    }
    
    static void shutdown() {
        if (initialized) {
            auto publisher = Telemetry::getPublisher();
            publisher->shutdown();
            initialized = false;
        }
    }
    
    static bool isInitialized() {
        return initialized;
    }
};

bool TelemetryInit::initialized = false;

// Auto-initialization helper - ensures telemetry is initialized on first use
class TelemetryAutoInit {
public:
    TelemetryAutoInit() {
        TelemetryInit::initialize();
    }
};

// Global instance to ensure auto-initialization
static TelemetryAutoInit _telemetry_auto_init;

// Convenience namespace aliases
namespace Tel = Telemetry;

// Additional helper functions for common patterns
namespace TelemetryHelpers {
    
    // Log a 3D position with automatic naming
    inline void logPosition(const std::string& prefix, double x, double y, double z, const std::string& unit = "m") {
        TEL_VECTOR(prefix + "/position", x, y, z);
        TEL_NUMBER(prefix + "/position_x", x);
        TEL_NUMBER(prefix + "/position_y", y);
        TEL_NUMBER(prefix + "/position_z", z);
        TEL_NUMBER(prefix + "/position_magnitude", sqrt(x*x + y*y + z*z));
    }
    
    // Log a 3D velocity with automatic naming
    inline void logVelocity(const std::string& prefix, double vx, double vy, double vz, const std::string& unit = "m/s") {
        TEL_VECTOR(prefix + "/velocity", vx, vy, vz);
        TEL_NUMBER(prefix + "/velocity_x", vx);
        TEL_NUMBER(prefix + "/velocity_y", vy);
        TEL_NUMBER(prefix + "/velocity_z", vz);
        TEL_NUMBER(prefix + "/velocity_magnitude", sqrt(vx*vx + vy*vy + vz*vz));
    }
    
    // Log error between two 3D vectors
    inline void logError(const std::string& prefix, 
                        double ref_x, double ref_y, double ref_z,
                        double actual_x, double actual_y, double actual_z,
                        const std::string& unit = "m") {
        double ex = ref_x - actual_x;
        double ey = ref_y - actual_y; 
        double ez = ref_z - actual_z;
        
        TEL_VECTOR(prefix + "/error", ex, ey, ez);
        TEL_NUMBER(prefix + "/error_x", ex);
        TEL_NUMBER(prefix + "/error_y", ey);
        TEL_NUMBER(prefix + "/error_z", ez);
        TEL_NUMBER(prefix + "/error_magnitude", sqrt(ex*ex + ey*ey + ez*ez));
    }
    
    // Log system health metrics
    inline void logHealth(const std::string& component, bool isHealthy, const std::string& status = "") {
        TEL_BOOL("health/" + component + "/healthy", isHealthy);
        if (!status.empty()) {
            TEL_STRING("health/" + component + "/status", status);
        }
        TEL_NUMBER("health/" + component + "/last_check", time(nullptr));
    }
    
    // Log performance metrics for a component
    inline void logPerformance(const std::string& component, double fps, double avgFrameTime, double maxFrameTime) {
        TEL_FPS(component, fps);
        TEL_FRAME_TIME(component, avgFrameTime);
        TEL_NUMBER("performance/" + component + "/max_frame_time", maxFrameTime);
        TEL_NUMBER("performance/" + component + "/load_percent", (avgFrameTime / (1000.0/fps)) * 100.0);
    }
    
    // Log memory usage (if available)
    inline void logMemoryUsage(const std::string& component, size_t bytesUsed, size_t maxBytes = 0) {
        TEL_NUMBER("memory/" + component + "/bytes_used", static_cast<double>(bytesUsed));
        if (maxBytes > 0) {
            TEL_NUMBER("memory/" + component + "/bytes_max", static_cast<double>(maxBytes));
            TEL_NUMBER("memory/" + component + "/usage_percent", (static_cast<double>(bytesUsed) / maxBytes) * 100.0);
        }
    }
}

// Shortened namespace for helpers
namespace TelH = TelemetryHelpers;

#endif // TELEMETRY_H
