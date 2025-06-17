#include "telemetry/TelemetryEntry.h"
#include <algorithm>

// TelemetryEntry implementation
TelemetryEntry::TelemetryEntry(const std::string& entryName, 
                              const std::string& entryCategory,
                              const std::string& entryUnit,
                              size_t maxHistory)
    : name(entryName), category(entryCategory), unit(entryUnit), 
      maxHistorySize(maxHistory), enabled(true) {
}

void TelemetryEntry::addData(const TelemetryValue& value) {
    if (!enabled) return;
    
    std::lock_guard<std::mutex> lock(dataMutex);
    
    history.emplace_back(value);
    
    // Maintain history size limit
    if (history.size() > maxHistorySize) {
        history.erase(history.begin(), history.begin() + (history.size() - maxHistorySize));
    }
}

TelemetryValue TelemetryEntry::getLatestValue() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    if (history.empty()) {
        return 0.0; // Default value
    }
    
    return history.back().value;
}

std::vector<TelemetryDataPoint> TelemetryEntry::getHistory() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return history; // Copy
}

std::vector<TelemetryDataPoint> TelemetryEntry::getDataSince(
    std::chrono::steady_clock::time_point since) const {
    
    std::lock_guard<std::mutex> lock(dataMutex);
    
    std::vector<TelemetryDataPoint> result;
    for (const auto& point : history) {
        if (point.timestamp >= since) {
            result.push_back(point);
        }
    }
    
    return result;
}

void TelemetryEntry::clearHistory() {
    std::lock_guard<std::mutex> lock(dataMutex);
    history.clear();
}

void TelemetryEntry::setEnabled(bool enable) {
    enabled = enable;
}

bool TelemetryEntry::isEnabled() const {
    return enabled;
}

size_t TelemetryEntry::getHistorySize() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    return history.size();
}

void TelemetryEntry::setCustomSerializer(std::function<json(const TelemetryValue&)> serializer) {
    customSerializer = serializer;
}

json TelemetryEntry::toJson(bool includeHistory) const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    json j;
    j["name"] = name;
    j["category"] = category;
    j["unit"] = unit;
    j["enabled"] = enabled;
    j["historySize"] = history.size();
    
    if (!history.empty()) {
        if (customSerializer) {
            j["latestValue"] = customSerializer(history.back().value);
        } else {
            j["latestValue"] = valueToJson(history.back().value);
        }
        
        // Convert timestamp to milliseconds since epoch
        auto timeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            history.back().timestamp.time_since_epoch()).count();
        j["latestTimestamp"] = timeMs;
    }
    
    if (includeHistory) {
        json historyArray = json::array();
        for (const auto& point : history) {
            json pointJson;
            if (customSerializer) {
                pointJson["value"] = customSerializer(point.value);
            } else {
                pointJson["value"] = valueToJson(point.value);
            }
            
            auto timeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                point.timestamp.time_since_epoch()).count();
            pointJson["timestamp"] = timeMs;
            
            historyArray.push_back(pointJson);
        }
        j["history"] = historyArray;
    }
    
    return j;
}

json TelemetryEntry::valueToJson(const TelemetryValue& value) {
    return std::visit([](const auto& v) -> json {
        return json(v);
    }, value);
}

// ScalarTelemetryEntry implementation
ScalarTelemetryEntry::ScalarTelemetryEntry(const std::string& name, 
                                          const std::string& category,
                                          const std::string& unit)
    : TelemetryEntry(name, category, unit) {
}

void ScalarTelemetryEntry::setValue(double value) {
    addData(value);
}

void ScalarTelemetryEntry::setValue(float value) {
    addData(static_cast<double>(value));
}

void ScalarTelemetryEntry::setValue(int value) {
    addData(value);
}

void ScalarTelemetryEntry::setValue(bool value) {
    addData(value);
}

double ScalarTelemetryEntry::getValueAsDouble() const {
    auto value = getLatestValue();
    return std::visit([](const auto& v) -> double {
        if constexpr (std::is_arithmetic_v<std::decay_t<decltype(v)>>) {
            return static_cast<double>(v);
        }
        return 0.0;
    }, value);
}

// VectorTelemetryEntry implementation
VectorTelemetryEntry::VectorTelemetryEntry(const std::string& name, 
                                          const std::string& category,
                                          const std::string& unit,
                                          size_t expectedSize)
    : TelemetryEntry(name, category, unit) {
}

void VectorTelemetryEntry::setValue(const std::vector<double>& value) {
    addData(value);
}

void VectorTelemetryEntry::setValue(const std::vector<float>& value) {
    std::vector<double> doubleVec(value.begin(), value.end());
    addData(doubleVec);
}

void VectorTelemetryEntry::setValue(double x, double y, double z) {
    addData(std::vector<double>{x, y, z});
}

std::vector<double> VectorTelemetryEntry::getValueAsVector() const {
    auto value = getLatestValue();
    if (std::holds_alternative<std::vector<double>>(value)) {
        return std::get<std::vector<double>>(value);
    }
    return {};
}

// TimestampedTelemetryEntry implementation
TimestampedTelemetryEntry::TimestampedTelemetryEntry(const std::string& name,
                                                    const std::string& category)
    : TelemetryEntry(name, category) {
}

void TimestampedTelemetryEntry::addEvent(const std::string& event) {
    addData(event);
}

void TimestampedTelemetryEntry::addMeasurement(double value, const std::string& description) {
    if (description.empty()) {
        addData(value);
    } else {
        // Store as string with description
        addData(description + ": " + std::to_string(value));
    }
}
