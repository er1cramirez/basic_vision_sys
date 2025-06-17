#include "telemetry/TelemetryPublisher.h"
#include "../tools/example_scope/udp_telemetry_server.h"
#include <iostream>
#include <algorithm>

// Static member initialization
std::shared_ptr<TelemetryPublisher> TelemetryPublisher::instance = nullptr;
std::mutex TelemetryPublisher::instanceMutex;

// Timer storage for timing utilities
static std::map<std::string, std::chrono::steady_clock::time_point> activeTimers;
static std::mutex timersMutex;

TelemetryPublisher::TelemetryPublisher() 
    : publishing(false), publishInterval(std::chrono::milliseconds(50)) {
}

TelemetryPublisher::~TelemetryPublisher() {
    shutdown();
}

std::shared_ptr<TelemetryPublisher> TelemetryPublisher::getInstance() {
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (!instance) {
        instance = std::shared_ptr<TelemetryPublisher>(new TelemetryPublisher());
    }
    return instance;
}

bool TelemetryPublisher::initialize(uint16_t udpPort, uint32_t publishRateHz) {
    try {
        // Create UDP server
        udpServer = std::make_shared<UDPTelemetryServer>(udpPort, publishRateHz * 2);
        
        if (!udpServer->start()) {
            std::cerr << "Failed to start UDP telemetry server on port " << udpPort << std::endl;
            return false;
        }
        
        // Set publish rate
        setPublishRate(publishRateHz);
        
        // Start publishing
        startPublishing();
        
        std::cout << "Telemetry publisher initialized on UDP port " << udpPort 
                  << " at " << publishRateHz << " Hz" << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing telemetry publisher: " << e.what() << std::endl;
        return false;
    }
}

void TelemetryPublisher::shutdown() {
    stopPublishing();
    
    if (udpServer) {
        udpServer->stop();
        udpServer.reset();
    }
    
    std::lock_guard<std::mutex> lock(entriesMutex);
    entries.clear();
}

std::shared_ptr<TelemetryEntry> TelemetryPublisher::getEntry(const std::string& name) {
    std::lock_guard<std::mutex> lock(entriesMutex);
    
    auto it = entries.find(name);
    if (it != entries.end()) {
        return it->second;
    }
    
    // Create new generic entry
    auto entry = std::make_shared<TelemetryEntry>(name);
    entries[name] = entry;
    return entry;
}

std::shared_ptr<ScalarTelemetryEntry> TelemetryPublisher::getScalarEntry(
    const std::string& name, const std::string& category, const std::string& unit) {
    
    std::lock_guard<std::mutex> lock(entriesMutex);
    
    auto it = entries.find(name);
    if (it != entries.end()) {
        // Try to cast to scalar entry
        auto scalarEntry = std::dynamic_pointer_cast<ScalarTelemetryEntry>(it->second);
        if (scalarEntry) {
            return scalarEntry;
        }
    }
    
    // Create new scalar entry
    auto entry = std::make_shared<ScalarTelemetryEntry>(name, category, unit);
    entries[name] = entry;
    return entry;
}

std::shared_ptr<VectorTelemetryEntry> TelemetryPublisher::getVectorEntry(
    const std::string& name, const std::string& category, const std::string& unit) {
    
    std::lock_guard<std::mutex> lock(entriesMutex);
    
    auto it = entries.find(name);
    if (it != entries.end()) {
        auto vectorEntry = std::dynamic_pointer_cast<VectorTelemetryEntry>(it->second);
        if (vectorEntry) {
            return vectorEntry;
        }
    }
    
    auto entry = std::make_shared<VectorTelemetryEntry>(name, category, unit);
    entries[name] = entry;
    return entry;
}

std::shared_ptr<TimestampedTelemetryEntry> TelemetryPublisher::getTimestampedEntry(
    const std::string& name, const std::string& category) {
    
    std::lock_guard<std::mutex> lock(entriesMutex);
    
    auto it = entries.find(name);
    if (it != entries.end()) {
        auto timestampedEntry = std::dynamic_pointer_cast<TimestampedTelemetryEntry>(it->second);
        if (timestampedEntry) {
            return timestampedEntry;
        }
    }
    
    auto entry = std::make_shared<TimestampedTelemetryEntry>(name, category);
    entries[name] = entry;
    return entry;
}

void TelemetryPublisher::removeEntry(const std::string& name) {
    std::lock_guard<std::mutex> lock(entriesMutex);
    entries.erase(name);
}

std::vector<std::shared_ptr<TelemetryEntry>> TelemetryPublisher::getAllEntries() const {
    std::lock_guard<std::mutex> lock(entriesMutex);
    
    std::vector<std::shared_ptr<TelemetryEntry>> result;
    for (const auto& pair : entries) {
        result.push_back(pair.second);
    }
    return result;
}

std::vector<std::shared_ptr<TelemetryEntry>> TelemetryPublisher::getEntriesByCategory(
    const std::string& category) const {
    
    std::lock_guard<std::mutex> lock(entriesMutex);
    
    std::vector<std::shared_ptr<TelemetryEntry>> result;
    for (const auto& pair : entries) {
        if (pair.second->getCategory() == category) {
            result.push_back(pair.second);
        }
    }
    return result;
}

void TelemetryPublisher::logScalar(const std::string& name, double value, const std::string& unit) {
    auto entry = getScalarEntry(name, "scalars", unit);
    entry->setValue(value);
}

void TelemetryPublisher::logVector(const std::string& name, const std::vector<double>& value, const std::string& unit) {
    auto entry = getVectorEntry(name, "vectors", unit);
    entry->setValue(value);
}

void TelemetryPublisher::logVector(const std::string& name, double x, double y, double z, const std::string& unit) {
    auto entry = getVectorEntry(name, "vectors", unit);
    entry->setValue(x, y, z);
}

void TelemetryPublisher::logEvent(const std::string& name, const std::string& event) {
    auto entry = getTimestampedEntry(name, "events");
    entry->addEvent(event);
}

void TelemetryPublisher::startPublishing() {
    if (publishing) return;
    
    publishing = true;
    publishThread = std::thread(&TelemetryPublisher::publishLoop, this);
}

void TelemetryPublisher::stopPublishing() {
    if (!publishing) return;
    
    publishing = false;
    if (publishThread.joinable()) {
        publishThread.join();
    }
}

bool TelemetryPublisher::isPublishing() const {
    return publishing;
}

void TelemetryPublisher::setPublishRate(uint32_t rateHz) {
    if (rateHz > 0) {
        publishInterval = std::chrono::milliseconds(1000 / rateHz);
    }
}

void TelemetryPublisher::publishNow() {
    if (!udpServer) return;
    
    try {
        json telemetryData;
        telemetryData["type"] = "telemetry_update";
        telemetryData["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        json entriesJson = json::object();
        
        {
            std::lock_guard<std::mutex> lock(entriesMutex);
            for (const auto& pair : entries) {
                if (pair.second->isEnabled() && pair.second->getHistorySize() > 0) {
                    entriesJson[pair.first] = pair.second->toJson(false); // Only latest values
                }
            }
        }
        
        telemetryData["entries"] = entriesJson;
        telemetryData["entryCount"] = entriesJson.size();
        
        // Serialize and send using the generic telemetry method
        udpServer->queueGenericTelemetry(telemetryData);
        
    } catch (const std::exception& e) {
        std::cerr << "Error publishing telemetry data: " << e.what() << std::endl;
    }
}

void TelemetryPublisher::publishLoop() {
    while (publishing) {
        publishNow();
        std::this_thread::sleep_for(publishInterval);
    }
}

size_t TelemetryPublisher::getEntryCount() const {
    std::lock_guard<std::mutex> lock(entriesMutex);
    return entries.size();
}

std::vector<std::string> TelemetryPublisher::getEntryNames() const {
    std::lock_guard<std::mutex> lock(entriesMutex);
    
    std::vector<std::string> names;
    for (const auto& pair : entries) {
        names.push_back(pair.first);
    }
    return names;
}

void TelemetryPublisher::clearAllData() {
    std::lock_guard<std::mutex> lock(entriesMutex);
    for (auto& pair : entries) {
        pair.second->clearHistory();
    }
}

json TelemetryPublisher::exportAllData(bool includeHistory) const {
    json result;
    result["exportTimestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    json entriesJson = json::object();
    
    {
        std::lock_guard<std::mutex> lock(entriesMutex);
        for (const auto& pair : entries) {
            entriesJson[pair.first] = pair.second->toJson(includeHistory);
        }
    }
    
    result["entries"] = entriesJson;
    return result;
}

json TelemetryPublisher::exportCategory(const std::string& category, bool includeHistory) const {
    json result;
    result["category"] = category;
    result["exportTimestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    json entriesJson = json::object();
    
    {
        std::lock_guard<std::mutex> lock(entriesMutex);
        for (const auto& pair : entries) {
            if (pair.second->getCategory() == category) {
                entriesJson[pair.first] = pair.second->toJson(includeHistory);
            }
        }
    }
    
    result["entries"] = entriesJson;
    return result;
}

// Global convenience functions implementation
namespace Telemetry {
    
std::shared_ptr<TelemetryPublisher> getPublisher() {
    return TelemetryPublisher::getInstance();
}

void putNumber(const std::string& key, double value) {
    auto publisher = getPublisher();
    publisher->logScalar(key, value);
}

void putString(const std::string& key, const std::string& value) {
    auto publisher = getPublisher();
    auto entry = publisher->getEntry(key);
    entry->addData(value);
}

void putBoolean(const std::string& key, bool value) {
    auto publisher = getPublisher();
    auto entry = publisher->getScalarEntry(key);
    entry->setValue(value);
}

void putNumberArray(const std::string& key, const std::vector<double>& value) {
    auto publisher = getPublisher();
    publisher->logVector(key, value);
}

void putVector3(const std::string& key, double x, double y, double z) {
    auto publisher = getPublisher();
    publisher->logVector(key, x, y, z);
}

double getNumber(const std::string& key, double defaultValue) {
    auto publisher = getPublisher();
    auto entry = publisher->getScalarEntry(key);
    try {
        return entry->getValueAsDouble();
    } catch (...) {
        return defaultValue;
    }
}

std::string getString(const std::string& key, const std::string& defaultValue) {
    auto publisher = getPublisher();
    auto entry = publisher->getEntry(key);
    try {
        auto value = entry->getLatestValue();
        if (std::holds_alternative<std::string>(value)) {
            return std::get<std::string>(value);
        }
    } catch (...) {}
    return defaultValue;
}

bool getBoolean(const std::string& key, bool defaultValue) {
    auto publisher = getPublisher();
    auto entry = publisher->getEntry(key);
    try {
        auto value = entry->getLatestValue();
        if (std::holds_alternative<bool>(value)) {
            return std::get<bool>(value);
        }
    } catch (...) {}
    return defaultValue;
}

std::vector<double> getNumberArray(const std::string& key) {
    auto publisher = getPublisher();
    auto entry = publisher->getVectorEntry(key);
    return entry->getValueAsVector();
}

void logEvent(const std::string& event) {
    auto publisher = getPublisher();
    publisher->logEvent("system_events", event);
}

void logError(const std::string& error) {
    auto publisher = getPublisher();
    publisher->logEvent("errors", "ERROR: " + error);
}

void logInfo(const std::string& info) {
    auto publisher = getPublisher();
    publisher->logEvent("info", "INFO: " + info);
}

void logWarning(const std::string& warning) {
    auto publisher = getPublisher();
    publisher->logEvent("warnings", "WARNING: " + warning);
}

void startTimer(const std::string& name) {
    std::lock_guard<std::mutex> lock(timersMutex);
    activeTimers[name] = std::chrono::steady_clock::now();
}

void endTimer(const std::string& name) {
    std::lock_guard<std::mutex> lock(timersMutex);
    auto it = activeTimers.find(name);
    if (it != activeTimers.end()) {
        auto duration = std::chrono::steady_clock::now() - it->second;
        auto durationMs = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
        
        auto publisher = getPublisher();
        publisher->logScalar("timers/" + name, durationMs, "ms");
        
        activeTimers.erase(it);
    }
}

void recordFrameTime(const std::string& component, double timeMs) {
    auto publisher = getPublisher();
    publisher->logScalar("performance/" + component + "_frame_time", timeMs, "ms");
}

void recordFPS(const std::string& component, double fps) {
    auto publisher = getPublisher();
    publisher->logScalar("performance/" + component + "_fps", fps, "fps");
}

} // namespace Telemetry
