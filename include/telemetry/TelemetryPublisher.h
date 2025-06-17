#ifndef TELEMETRY_PUBLISHER_H
#define TELEMETRY_PUBLISHER_H

#include "telemetry/TelemetryEntry.h"
#include <map>
#include <memory>
#include <thread>
#include <atomic>
#include <functional>
#include <chrono>

// Forward declarations
class UDPTelemetryServer;

// Telemetry publisher interface - inspired by WPILib Sendable
class TelemetryPublisher {
private:
    std::map<std::string, std::shared_ptr<TelemetryEntry>> entries;
    std::mutex entriesMutex;
    
    // Publication thread
    std::thread publishThread;
    std::atomic<bool> publishing;
    std::chrono::milliseconds publishInterval;
    
    // UDP server for transmission
    std::shared_ptr<UDPTelemetryServer> udpServer;
    
    // Global instance
    static std::shared_ptr<TelemetryPublisher> instance;
    static std::mutex instanceMutex;
    
    // Private constructor for singleton
    TelemetryPublisher();
    
    // Publication loop
    void publishLoop();

public:
    ~TelemetryPublisher();
    
    // Get singleton instance
    static std::shared_ptr<TelemetryPublisher> getInstance();
    
    // Initialize with UDP server
    bool initialize(uint16_t udpPort = 14559, uint32_t publishRateHz = 20);
    
    // Shutdown
    void shutdown();
    
    // Entry management
    std::shared_ptr<TelemetryEntry> getEntry(const std::string& name);
    std::shared_ptr<ScalarTelemetryEntry> getScalarEntry(const std::string& name, 
                                                         const std::string& category = "scalars",
                                                         const std::string& unit = "");
    std::shared_ptr<VectorTelemetryEntry> getVectorEntry(const std::string& name,
                                                         const std::string& category = "vectors", 
                                                         const std::string& unit = "");
    std::shared_ptr<TimestampedTelemetryEntry> getTimestampedEntry(const std::string& name,
                                                                  const std::string& category = "events");
    
    // Generic entry creation
    template<typename T>
    std::shared_ptr<T> createEntry(const std::string& name, 
                                  const std::string& category = "default",
                                  const std::string& unit = "") {
        std::lock_guard<std::mutex> lock(entriesMutex);
        
        auto entry = std::make_shared<T>(name, category, unit);
        entries[name] = entry;
        return entry;
    }
    
    // Remove entry
    void removeEntry(const std::string& name);
    
    // Get all entries
    std::vector<std::shared_ptr<TelemetryEntry>> getAllEntries() const;
    
    // Get entries by category
    std::vector<std::shared_ptr<TelemetryEntry>> getEntriesByCategory(const std::string& category) const;
    
    // Quick data logging functions (convenience methods)
    void logScalar(const std::string& name, double value, const std::string& unit = "");
    void logVector(const std::string& name, const std::vector<double>& value, const std::string& unit = "");
    void logVector(const std::string& name, double x, double y, double z, const std::string& unit = "");
    void logEvent(const std::string& name, const std::string& event);
    
    // Enable/disable publishing
    void startPublishing();
    void stopPublishing();
    bool isPublishing() const;
    
    // Set publish rate
    void setPublishRate(uint32_t rateHz);
    
    // Manual publish trigger
    void publishNow();
    
    // Get statistics
    size_t getEntryCount() const;
    std::vector<std::string> getEntryNames() const;
    
    // Clear all data
    void clearAllData();
    
    // JSON export
    json exportAllData(bool includeHistory = false) const;
    json exportCategory(const std::string& category, bool includeHistory = false) const;
};

// Global convenience functions (similar to WPILib SmartDashboard)
namespace Telemetry {
    // Get publisher instance
    std::shared_ptr<TelemetryPublisher> getPublisher();
    
    // Quick logging functions
    void putNumber(const std::string& key, double value);
    void putString(const std::string& key, const std::string& value);
    void putBoolean(const std::string& key, bool value);
    void putNumberArray(const std::string& key, const std::vector<double>& value);
    void putVector3(const std::string& key, double x, double y, double z);
    
    // Get functions
    double getNumber(const std::string& key, double defaultValue = 0.0);
    std::string getString(const std::string& key, const std::string& defaultValue = "");
    bool getBoolean(const std::string& key, bool defaultValue = false);
    std::vector<double> getNumberArray(const std::string& key);
    
    // Event logging
    void logEvent(const std::string& event);
    void logError(const std::string& error);
    void logInfo(const std::string& info);
    void logWarning(const std::string& warning);
    
    // Timing utilities
    void startTimer(const std::string& name);
    void endTimer(const std::string& name);
    
    // Performance monitoring
    void recordFrameTime(const std::string& component, double timeMs);
    void recordFPS(const std::string& component, double fps);
}

#endif // TELEMETRY_PUBLISHER_H
