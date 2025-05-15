#include "Logger.h"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <cstring>
#include <cstdarg>
#include <filesystem>
#include <sstream>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#include <sys/types.h>
#define MKDIR(dir) mkdir(dir, 0755)
#endif

Logger::Logger() : initialized(false) {
    startTime = std::chrono::steady_clock::now();
}

Logger::~Logger() {
    close();
}

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

std::string Logger::generateTimestampString() const {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    
    // Format: YYYYMMDD_HHMMSS
    std::tm now_tm;
    #ifdef _WIN32
    localtime_s(&now_tm, &now_time_t);
    #else
    localtime_r(&now_time_t, &now_tm);
    #endif
    
    ss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
    
    return ss.str();
}

bool Logger::createLogDirectory(const std::string& basePath, const std::string& sessionName) {
    std::string timestamp = generateTimestampString();
    
    // Create base logs directory if it doesn't exist
    if (MKDIR(basePath.c_str()) != 0 && errno != EEXIST) {
        std::cerr << "Error: Failed to create base log directory: " << basePath << std::endl;
        return false;
    }
    
    // Create session directory with timestamp
    if (sessionName.empty()) {
        logDirectory = basePath + "/log_" + timestamp;
    } else {
        logDirectory = basePath + "/" + sessionName + "_" + timestamp;
    }
    
    if (MKDIR(logDirectory.c_str()) != 0) {
        std::cerr << "Error: Failed to create log directory: " << logDirectory << std::endl;
        return false;
    }
    
    return true;
}

bool Logger::initialize(const std::string& dirPath, const std::string& sessionName) {
    std::lock_guard<std::mutex> lock(logMutex);
    
    if (initialized) {
        return true; // Already initialized
    }
    
    // Create log directory
    if (!createLogDirectory(dirPath, sessionName)) {
        return false;
    }
    
    // Create description file
    std::string descFilePath = logDirectory + "/description.txt";
    descriptionFile.open(descFilePath, std::ios::out);
    if (!descriptionFile.is_open()) {
        std::cerr << "Error: Failed to create description file: " << descFilePath << std::endl;
        return false;
    }
    
    // Write header to description file
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    
    descriptionFile << "UAV Vision Control System Log Description" << std::endl;
    descriptionFile << "Created: " << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << std::endl;
    descriptionFile << "Format Specifiers:" << std::endl;
    descriptionFile << "  Q = uint64_t, q = int64_t" << std::endl;
    descriptionFile << "  I = uint32_t, i = int32_t" << std::endl;
    descriptionFile << "  H = uint16_t, h = int16_t" << std::endl;
    descriptionFile << "  B = uint8_t, b = int8_t" << std::endl;
    descriptionFile << "  f = float, d = double" << std::endl;
    descriptionFile << "  N = char[16], Z = char[64]" << std::endl;
    descriptionFile << "-------------------------------------------------------------------" << std::endl;
    descriptionFile << "Message Types:" << std::endl;
    
    initialized = true;
    std::cout << "Logger initialized in directory: " << logDirectory << std::endl;
    
    return true;
}

void Logger::writeFileHeader(std::ofstream& file, const std::string& msgName, const std::string& fieldNames) {
    // Write header (field names)
    file << fieldNames << std::endl;
}

std::ofstream& Logger::getMessageFile(const std::string& msgName, const std::string& fieldNames) {
    auto it = messageFiles.find(msgName);
    
    if (it == messageFiles.end()) {
        // Need to create a new file for this message type
        std::string filePath = logDirectory + "/" + msgName + ".csv";
        messageFiles[msgName].open(filePath, std::ios::out);
        
        if (!messageFiles[msgName].is_open()) {
            std::cerr << "Error: Failed to create message file: " << filePath << std::endl;
            throw std::runtime_error("Failed to create message file");
        }
        
        // Write header for the new file
        writeFileHeader(messageFiles[msgName], msgName, fieldNames);
    }
    
    return messageFiles[msgName];
}

void Logger::updateDescriptionFile(const std::string& msgName, const std::string& fieldNames, const std::string& formatStr) {
    // Only add to description file if this is a new message type
    auto it = messageDescriptions.find(msgName);
    if (it == messageDescriptions.end()) {
        // Add to tracked message descriptions
        messageDescriptions[msgName] = MessageDescription(msgName, fieldNames, formatStr);
        
        // Add to description file
        descriptionFile << msgName << ": " << fieldNames << " (" << formatStr << ")" << std::endl;
        descriptionFile.flush();
        
        std::cout << "Added new message type: " << msgName << std::endl;
    }
}

bool Logger::isInitialized() const {
    std::lock_guard<std::mutex> lock(logMutex);
    return initialized;
}

uint64_t Logger::getMicroseconds() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count();
}

std::string Logger::getLogDirectory() const {
    std::lock_guard<std::mutex> lock(logMutex);
    return logDirectory;
}

bool Logger::Write(const char* msgName, const char* fieldNames, const char* formatStr, ...) {
    std::lock_guard<std::mutex> lock(logMutex);
    
    if (!initialized) {
        std::cerr << "Error: Logger not initialized" << std::endl;
        return false;
    }
    
    // Check if msgName is valid (4 characters)
    if (strlen(msgName) != 4) {
        std::cerr << "Error: Message name must be exactly 4 characters: " << msgName << std::endl;
        return false;
    }
    
    try {
        // Update description file if needed
        updateDescriptionFile(msgName, fieldNames, formatStr);
        
        // Get or create file for this message type
        std::ofstream& messageFile = getMessageFile(msgName, fieldNames);
        
        // Process variable arguments based on format string
        va_list args;
        va_start(args, formatStr);
        
        size_t formatLen = strlen(formatStr);
        for (size_t i = 0; i < formatLen; i++) {
            // Add comma before all fields except the first
            if (i > 0) {
                messageFile << ",";
            }
            
            char format = formatStr[i];
            switch (format) {
                case 'Q': { // uint64_t
                    uint64_t val = va_arg(args, uint64_t);
                    messageFile << val;
                    break;
                }
                case 'q': { // int64_t
                    int64_t val = va_arg(args, int64_t);
                    messageFile << val;
                    break;
                }
                case 'f': { // float (promoted to double in va_arg)
                    float val = static_cast<float>(va_arg(args, double));
                    messageFile << std::fixed << std::setprecision(6) << val;
                    break;
                }
                case 'd': { // double
                    double val = va_arg(args, double);
                    messageFile << std::fixed << std::setprecision(12) << val;
                    break;
                }
                case 'I': { // uint32_t
                    uint32_t val = va_arg(args, uint32_t);
                    messageFile << val;
                    break;
                }
                case 'i': { // int32_t
                    int32_t val = va_arg(args, int32_t);
                    messageFile << val;
                    break;
                }
                case 'H': { // uint16_t (promoted to int in va_arg)
                    uint16_t val = static_cast<uint16_t>(va_arg(args, int));
                    messageFile << val;
                    break;
                }
                case 'h': { // int16_t (promoted to int in va_arg)
                    int16_t val = static_cast<int16_t>(va_arg(args, int));
                    messageFile << val;
                    break;
                }
                case 'B': { // uint8_t (promoted to int in va_arg)
                    uint8_t val = static_cast<uint8_t>(va_arg(args, int));
                    messageFile << static_cast<int>(val); // Print as number, not character
                    break;
                }
                case 'b': { // int8_t (promoted to int in va_arg)
                    int8_t val = static_cast<int8_t>(va_arg(args, int));
                    messageFile << static_cast<int>(val); // Print as number, not character
                    break;
                }
                case 'N': { // char[16]
                    const char* val = va_arg(args, const char*);
                    messageFile << "\"" << (val ? val : "") << "\"";
                    break;
                }
                case 'Z': { // char[64]
                    const char* val = va_arg(args, const char*);
                    messageFile << "\"" << (val ? val : "") << "\"";
                    break;
                }
                default:
                    messageFile << "UNKNOWN_FORMAT";
                    std::cerr << "Error: Unknown format specifier '" << format << "'" << std::endl;
                    break;
            }
        }
        
        va_end(args);
        
        // End the log entry
        messageFile << std::endl;
        
        // Flush to ensure data is written even if program crashes
        messageFile.flush();
        
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error writing log message: " << e.what() << std::endl;
        return false;
    }
}

void Logger::close() {
    std::lock_guard<std::mutex> lock(logMutex);
    
    if (initialized) {
        // Close all message files
        for (auto& [name, file] : messageFiles) {
            if (file.is_open()) {
                file.close();
            }
        }
        messageFiles.clear();
        
        // Close description file
        if (descriptionFile.is_open()) {
            descriptionFile.close();
        }
        
        std::cout << "Logger closed. Files saved in: " << logDirectory << std::endl;
    }
    
    initialized = false;
}