#ifndef UDP_TELEMETRY_SERVER_H
#define UDP_TELEMETRY_SERVER_H

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <chrono>
#include <queue>
#include <condition_variable> // Added missing include for std::condition_variable
#include <nlohmann/json.hpp>  // Include JSON library for serialization

// Socket headers for different platforms
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef int socklen_t;
#else
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <fcntl.h>
#endif

// Include required control and vision headers instead of forward declarations
// #include "control_manager.h" // For ControlResultHandler and ControlResult
// #include "base_pipeline_result.h" // For PipelineResult

using json = nlohmann::json;

// Message types - MUST MATCH PYTHON CLIENT ENUM VALUES
enum TelemetryMessageType {
    TELEMETRY_CONTROL = 1,
    TELEMETRY_VISION = 2,    // Renamed back to VISION to match client
    TELEMETRY_SYSTEM = 3,    // Changed back to 3 to match client
    TELEMETRY_COMMAND = 4,   // Changed to 4 to avoid conflict
    TELEMETRY_GENERIC = 5    // New generic telemetry type
};

// Base telemetry message
struct TelemetryMessage {
    uint32_t messageType;
    uint64_t timestamp;
    uint32_t sequenceNumber;
    std::vector<uint8_t> payload;
};

class UDPTelemetryServer {
private:
    // Socket handling
    int socketFd;
    struct sockaddr_in serverAddr;
    uint16_t port;
    std::atomic<bool> running;
    
    // Threading
    std::thread sendThread;
    std::mutex queueMutex;
    std::condition_variable queueCondition;
    std::queue<TelemetryMessage> messageQueue;
    
    // Statistics
    std::atomic<uint64_t> messagesSent;
    std::atomic<uint64_t> bytesSent;
    std::atomic<uint32_t> sequenceCounter;
    
    // Rate limiting
    uint32_t maxQueueSize;
    uint32_t maxMessagesPerSecond;
    std::chrono::milliseconds sendInterval;
    
    // Initialize socket
    bool initializeSocket();
    
    // Close socket
    void closeSocket();
    
    // Sender thread function
    void senderLoop();
    
public:
    UDPTelemetryServer(uint16_t port = 14550, 
                     uint32_t maxRate = 50,  // Max 50 messages per second
                     uint32_t maxQueue = 100);
    
    ~UDPTelemetryServer();
    
    // Start the server
    bool start();
    
    // Stop the server
    void stop();
    
    // Queue a message for sending
    bool queueMessage(TelemetryMessageType type, const std::vector<uint8_t>& payload);
    
    // Helper to queue control result (commented out for now)
    // bool queueControlResult(const ControlResult& result);
    
    // Helper to queue vision result (commented out for now)
    // bool queueVisionResult(const PipelineResult& result);
    
    // Helper to queue system status
    bool queueSystemStatus(const std::string& status);
    
    // Helper to queue generic telemetry data
    bool queueGenericTelemetry(const json& data);
    
    // Get port
    uint16_t getPort() const;
    
    // Get statistics
    uint64_t getMessagesSent() const;
    uint64_t getBytesSent() const;
};

// Control result handler for UDP telemetry (commented out for now)
/*
class UDPControlTelemetryHandler : public ControlResultHandler {
private:
    std::shared_ptr<UDPTelemetryServer> telemetryServer;
    bool sendAllFrames;
    int sendEveryNFrames;
    uint64_t frameCounter;
    
public:
    UDPControlTelemetryHandler(std::shared_ptr<UDPTelemetryServer> server, 
                             bool sendAll = false,
                             int sendEveryN = 10);
    
    void handleResult(const ControlResult& result) override;
    
    // Configure sending rate
    void setSendRate(bool sendAll, int sendEveryN = 10);
};
*/

#endif // UDP_TELEMETRY_SERVER_H