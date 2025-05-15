#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
#include <mutex>
#include <map>
#include <chrono>
#include <vector>
#include <memory>
#include <cstdarg>
#include <unordered_map>

/**
 * @struct MessageDescription
 * @brief Describes a message type for logging
 */
struct MessageDescription {
    std::string name;           // Message name (4 characters)
    std::string fieldNames;     // Comma-separated field names
    std::string formatString;   // Format string for fields
    
    MessageDescription() = default;
    
    MessageDescription(const std::string& name, const std::string& fieldNames, const std::string& formatString)
        : name(name), fieldNames(fieldNames), formatString(formatString) {}
};

/**
 * @class Logger
 * @brief Singleton class that provides logging functionality for the UAV system
 * 
 * This logger follows a similar format to Ardupilot's logger, with message types
 * containing multiple fields with defined formats. Each message type is saved in
 * its own CSV file for easier processing.
 */
class Logger {
public:
    /**
     * @brief Get the singleton instance of the logger
     * @return Reference to the logger instance
     */
    static Logger& getInstance();

    /**
     * @brief Initialize the logger with a directory path
     * @param dirPath Path where the log directory will be created
     * @param sessionName Optional name for the logging session (default: timestamp)
     * @return True if initialization succeeded, false otherwise
     */
    bool initialize(const std::string& dirPath, const std::string& sessionName = "");

    /**
     * @brief Check if the logger is initialized
     * @return True if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Write a log message with variable arguments
     * @param msgName 4-character message name
     * @param fieldNames Comma-separated list of field names
     * @param formatStr Format string where:
     *        Q = uint64_t, q = int64_t, f = float, d = double,
     *        I = uint32_t, i = int32_t, H = uint16_t, h = int16_t,
     *        B = uint8_t, b = int8_t, N = char[16], Z = char[64]
     * @param ... Variable arguments matching the format string
     * @return True if write succeeded, false otherwise
     */
    bool Write(const char* msgName, const char* fieldNames, const char* formatStr, ...);

    /**
     * @brief Get the current timestamp in microseconds
     * @return Timestamp in microseconds since system start
     */
    uint64_t getMicroseconds() const;

    /**
     * @brief Get the path of the current log directory
     * @return Path to the current log directory
     */
    std::string getLogDirectory() const;

    /**
     * @brief Close the logger
     */
    void close();

    /**
     * @brief Destructor
     */
    ~Logger();

private:
    // Private constructor for singleton
    Logger();
    
    // Delete copy constructor and assignment operator
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Log files for each message type
    std::unordered_map<std::string, std::ofstream> messageFiles;
    
    // Description file
    std::ofstream descriptionFile;
    
    // Mutex for thread safety
    mutable std::mutex logMutex;
    
    // Flag indicating if logger is initialized
    bool initialized;
    
    // Start time for timestamp calculations
    std::chrono::time_point<std::chrono::steady_clock> startTime;
    
    // Log directory path
    std::string logDirectory;
    
    // Tracked message descriptions
    std::unordered_map<std::string, MessageDescription> messageDescriptions;
    
    // Helper function to generate a timestamp string
    std::string generateTimestampString() const;
    
    // Helper function to create the log directory
    bool createLogDirectory(const std::string& basePath, const std::string& sessionName);
    
    // Helper function to write log file headers
    void writeFileHeader(std::ofstream& file, const std::string& msgName, const std::string& fieldNames);
    
    // Helper function to write the description file
    void writeDescriptionFile();
    
    // Helper function to update the description file with a new message type
    void updateDescriptionFile(const std::string& msgName, const std::string& fieldNames, const std::string& formatStr);
    
    // Helper function to ensure a message file exists and has a header
    std::ofstream& getMessageFile(const std::string& msgName, const std::string& fieldNames);
};

// Global logger access function (similar to AP::logger())
namespace UAV {
    inline Logger& logger() {
        return Logger::getInstance();
    }
}

#endif // LOGGER_H