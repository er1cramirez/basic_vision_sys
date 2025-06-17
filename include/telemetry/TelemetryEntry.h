#ifndef TELEMETRY_ENTRY_H
#define TELEMETRY_ENTRY_H

#include <string>
#include <vector>
#include <variant>
#include <chrono>
#include <mutex>
#include <memory>
#include <functional>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Supported telemetry data types
using TelemetryValue = std::variant<
    double,
    float,
    int,
    long,
    bool,
    std::string,
    std::vector<double>,
    std::vector<float>,
    std::vector<int>
>;

// Telemetry data point with timestamp
struct TelemetryDataPoint {
    TelemetryValue value;
    std::chrono::steady_clock::time_point timestamp;
    
    TelemetryDataPoint(const TelemetryValue& val) 
        : value(val), timestamp(std::chrono::steady_clock::now()) {}
};

// Base class for telemetry entries
class TelemetryEntry {
private:
    std::string name;
    std::string category;
    std::string unit;
    std::vector<TelemetryDataPoint> history;
    mutable std::mutex dataMutex;  // Make mutable so it can be used in const methods
    size_t maxHistorySize;
    bool enabled;
    
    // Optional custom serializer
    std::function<json(const TelemetryValue&)> customSerializer;

public:
    TelemetryEntry(const std::string& entryName, 
                   const std::string& entryCategory = "default",
                   const std::string& entryUnit = "",
                   size_t maxHistory = 1000);
    
    // Add data to this entry
    void addData(const TelemetryValue& value);
    
    // Get latest value
    TelemetryValue getLatestValue() const;
    
    // Get all historical data (thread-safe copy)
    std::vector<TelemetryDataPoint> getHistory() const;
    
    // Get data since a specific time
    std::vector<TelemetryDataPoint> getDataSince(
        std::chrono::steady_clock::time_point since) const;
    
    // Clear history
    void clearHistory();
    
    // Enable/disable this entry
    void setEnabled(bool enable);
    bool isEnabled() const;
    
    // Getters
    const std::string& getName() const { return name; }
    const std::string& getCategory() const { return category; }
    const std::string& getUnit() const { return unit; }
    size_t getHistorySize() const;
    
    // Set custom serializer for complex data types
    void setCustomSerializer(std::function<json(const TelemetryValue&)> serializer);
    
    // Convert to JSON for transmission
    json toJson(bool includeHistory = false) const;
    
    // Static helper to convert TelemetryValue to JSON
    static json valueToJson(const TelemetryValue& value);
};

// Specialized entry types for common use cases
class ScalarTelemetryEntry : public TelemetryEntry {
public:
    ScalarTelemetryEntry(const std::string& name, 
                        const std::string& category = "scalars",
                        const std::string& unit = "");
    
    void setValue(double value);
    void setValue(float value);
    void setValue(int value);
    void setValue(bool value);
    
    double getValueAsDouble() const;
};

class VectorTelemetryEntry : public TelemetryEntry {
public:
    VectorTelemetryEntry(const std::string& name, 
                        const std::string& category = "vectors",
                        const std::string& unit = "",
                        size_t expectedSize = 3);
    
    void setValue(const std::vector<double>& value);
    void setValue(const std::vector<float>& value);
    void setValue(double x, double y, double z);
    
    std::vector<double> getValueAsVector() const;
};

class TimestampedTelemetryEntry : public TelemetryEntry {
public:
    TimestampedTelemetryEntry(const std::string& name,
                             const std::string& category = "timestamped");
    
    void addEvent(const std::string& event);
    void addMeasurement(double value, const std::string& description = "");
};

#endif // TELEMETRY_ENTRY_H
