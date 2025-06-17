#ifndef SIMPLE_TELEMETRY_H
#define SIMPLE_TELEMETRY_H

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <deque>
#include <variant>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <netinet/in.h>

using json = nlohmann::json;

// Simple telemetry value type
using TelemetryValue = std::variant<double, std::vector<double>, std::string, bool>;

// Simple telemetry data point
struct SimpleTelemetryPoint {
    TelemetryValue value;
    std::chrono::steady_clock::time_point timestamp;
    
    SimpleTelemetryPoint(const TelemetryValue& val) 
        : value(val), timestamp(std::chrono::steady_clock::now()) {}
};

// Simple telemetry manager
class SimpleTelemetryManager {
private:
    static std::shared_ptr<SimpleTelemetryManager> instance;
    static std::mutex instanceMutex;
    
    std::map<std::string, std::deque<SimpleTelemetryPoint>> data;
    mutable std::mutex dataMutex;
    std::atomic<bool> running{false};
    std::thread publishThread;
    
    // UDP socket for sending
    int sockfd;
    struct sockaddr_in serverAddr;
    
    void publishLoop();
    bool initUDP(uint16_t port);
    
public:
    SimpleTelemetryManager() = default;
    ~SimpleTelemetryManager();
    
    static std::shared_ptr<SimpleTelemetryManager> getInstance();
    
    bool initialize(uint16_t port = 14559, size_t maxHistory = 1000);
    void shutdown();
    
    void logNumber(const std::string& name, double value);
    void logVector(const std::string& name, double x, double y, double z);
    void logString(const std::string& name, const std::string& value);
    void logBool(const std::string& name, bool value);
    
    // Get data for external access
    std::map<std::string, std::vector<SimpleTelemetryPoint>> getAllData() const;
    
private:
    size_t maxHistorySize = 1000;
};

// Global convenience functions
namespace SimpleTelemetry {
    inline void putNumber(const std::string& key, double value) {
        SimpleTelemetryManager::getInstance()->logNumber(key, value);
    }
    
    inline void putVector3(const std::string& key, double x, double y, double z) {
        SimpleTelemetryManager::getInstance()->logVector(key, x, y, z);
    }
    
    inline void putString(const std::string& key, const std::string& value) {
        SimpleTelemetryManager::getInstance()->logString(key, value);
    }
    
    inline void putBoolean(const std::string& key, bool value) {
        SimpleTelemetryManager::getInstance()->logBool(key, value);
    }
    
    inline void logEvent(const std::string& event) {
        putString("events", event);
    }
    
    inline void logInfo(const std::string& info) {
        putString("info", "INFO: " + info);
    }
    
    inline void logWarning(const std::string& warning) {
        putString("warnings", "WARNING: " + warning);
    }
    
    inline void logError(const std::string& error) {
        putString("errors", "ERROR: " + error);
    }
    
    inline bool initialize(uint16_t port = 14559) {
        return SimpleTelemetryManager::getInstance()->initialize(port);
    }
    
    inline void shutdown() {
        SimpleTelemetryManager::getInstance()->shutdown();
    }
}

// Macros for easy usage
#define TEL_NUMBER(name, value) SimpleTelemetry::putNumber(name, value)
#define TEL_VECTOR(name, x, y, z) SimpleTelemetry::putVector3(name, x, y, z)
#define TEL_STRING(name, value) SimpleTelemetry::putString(name, value)
#define TEL_BOOL(name, value) SimpleTelemetry::putBoolean(name, value)
#define TEL_EVENT(message) SimpleTelemetry::logEvent(message)
#define TEL_INFO(message) SimpleTelemetry::logInfo(message)
#define TEL_WARNING(message) SimpleTelemetry::logWarning(message)
#define TEL_ERROR(message) SimpleTelemetry::logError(message)

// RAII Timer
class SimpleTelemetryTimer {
private:
    std::string name;
    std::chrono::steady_clock::time_point start;
    
public:
    explicit SimpleTelemetryTimer(const std::string& timerName) 
        : name(timerName), start(std::chrono::steady_clock::now()) {}
    
    ~SimpleTelemetryTimer() {
        auto duration = std::chrono::steady_clock::now() - start;
        auto ms = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
        TEL_NUMBER("timers/" + name, ms);
    }
};

#define TEL_AUTO_TIMER(name) SimpleTelemetryTimer _timer_##__LINE__(name)
#define TEL_FUNCTION_TIMER() TEL_AUTO_TIMER(__FUNCTION__)

// Helper functions
namespace TelH {
    inline void logPosition(const std::string& prefix, double x, double y, double z) {
        TEL_VECTOR(prefix + "/position", x, y, z);
        TEL_NUMBER(prefix + "/position_x", x);
        TEL_NUMBER(prefix + "/position_y", y);
        TEL_NUMBER(prefix + "/position_z", z);
        TEL_NUMBER(prefix + "/position_magnitude", sqrt(x*x + y*y + z*z));
    }
    
    inline void logVelocity(const std::string& prefix, double vx, double vy, double vz) {
        TEL_VECTOR(prefix + "/velocity", vx, vy, vz);
        TEL_NUMBER(prefix + "/velocity_x", vx);
        TEL_NUMBER(prefix + "/velocity_y", vy);
        TEL_NUMBER(prefix + "/velocity_z", vz);
        TEL_NUMBER(prefix + "/velocity_magnitude", sqrt(vx*vx + vy*vy + vz*vz));
    }
    
    inline void logHealth(const std::string& component, bool isHealthy, const std::string& status = "") {
        TEL_BOOL("health/" + component + "/healthy", isHealthy);
        if (!status.empty()) {
            TEL_STRING("health/" + component + "/status", status);
        }
    }
}

#endif // SIMPLE_TELEMETRY_H
