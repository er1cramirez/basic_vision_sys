#include "udp_telemetry_server.h"

// Initialize socket
bool UDPTelemetryServer::initializeSocket() {
    #ifdef _WIN32
        // Initialize Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "WSAStartup failed" << std::endl;
            return false;
        }
    #endif
    
    // Create UDP socket
    socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    // Setup server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);  // Bind to any interface
    serverAddr.sin_port = htons(port);
    
    // Set socket to non-blocking
    #ifdef _WIN32
        unsigned long mode = 1;
        if (ioctlsocket(socketFd, FIONBIO, &mode) != 0) {
            std::cerr << "Failed to set non-blocking mode" << std::endl;
            return false;
        }
    #else
        int flags = fcntl(socketFd, F_GETFL, 0);
        if (fcntl(socketFd, F_SETFL, flags | O_NONBLOCK) < 0) {
            std::cerr << "Failed to set non-blocking mode" << std::endl;
            return false;
        }
    #endif
    
    return true;
}

// Close socket
void UDPTelemetryServer::closeSocket() {
    #ifdef _WIN32
        closesocket(socketFd);
        WSACleanup();
    #else
        close(socketFd);
    #endif
}

// Sender thread function
void UDPTelemetryServer::senderLoop() {
    // Create broadcast address
    struct sockaddr_in broadcastAddr;
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_addr.s_addr = inet_addr("255.255.255.255");  // Broadcast address
    broadcastAddr.sin_port = htons(port);
    
    // Enable broadcast
    int broadcastEnable = 1;
    #ifdef _WIN32
        if (setsockopt(socketFd, SOL_SOCKET, SO_BROADCAST, (char*)&broadcastEnable, sizeof(broadcastEnable)) < 0) {
            std::cerr << "Failed to set broadcast option" << std::endl;
            return;
        }
    #else
        if (setsockopt(socketFd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
            std::cerr << "Failed to set broadcast option" << std::endl;
            return;
        }
    #endif
    
    auto lastSendTime = std::chrono::steady_clock::now();
    
    while (running) {
        // Rate limiting
        auto currentTime = std::chrono::steady_clock::now();
        auto timeSinceLastSend = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - lastSendTime);
        
        if (timeSinceLastSend < sendInterval) {
            // Sleep for the remaining time
            std::this_thread::sleep_for(sendInterval - timeSinceLastSend);
            continue;
        }
        
        // Get a message from the queue
        TelemetryMessage message;
        bool hasMessage = false;
        
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            // Wait for a message or timeout
            queueCondition.wait_for(lock, sendInterval, [this]() {
                return !messageQueue.empty() || !running;
            });
            
            if (!running) {
                break;
            }
            
            if (!messageQueue.empty()) {
                message = messageQueue.front();
                messageQueue.pop();
                hasMessage = true;
            }
        }
        
        if (hasMessage) {
            // Create the packet buffer
            std::vector<uint8_t> packet;
            
            // Add header (message type, timestamp, sequence number)
            uint32_t msgType = htonl(message.messageType);
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&msgType), 
                         reinterpret_cast<uint8_t*>(&msgType) + sizeof(msgType));
            
            uint64_t timestamp = htobe64(message.timestamp);
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&timestamp), 
                         reinterpret_cast<uint8_t*>(&timestamp) + sizeof(timestamp));
            
            uint32_t seqNum = htonl(message.sequenceNumber);
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&seqNum), 
                         reinterpret_cast<uint8_t*>(&seqNum) + sizeof(seqNum));
            
            // Add payload size
            uint32_t payloadSize = htonl(static_cast<uint32_t>(message.payload.size()));
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&payloadSize), 
                         reinterpret_cast<uint8_t*>(&payloadSize) + sizeof(payloadSize));
            
            // Add payload
            packet.insert(packet.end(), message.payload.begin(), message.payload.end());
            
            // Send the packet
            int bytesSent = sendto(socketFd, reinterpret_cast<const char*>(packet.data()), 
                                  packet.size(), 0, 
                                  reinterpret_cast<struct sockaddr*>(&broadcastAddr), 
                                  sizeof(broadcastAddr));
            
            if (bytesSent < 0) {
                std::cerr << "Failed to send telemetry packet" << std::endl;
            } else {
                messagesSent++;
                this->bytesSent += bytesSent;
            }
            
            lastSendTime = std::chrono::steady_clock::now();
        }
    }
}

// Constructor
UDPTelemetryServer::UDPTelemetryServer(uint16_t port,
                                     uint32_t maxRate,
                                     uint32_t maxQueue)
    : port(port),
      running(false),
      messagesSent(0),
      bytesSent(0),
      sequenceCounter(0),
      maxQueueSize(maxQueue),
      maxMessagesPerSecond(maxRate) {
    
    // Calculate send interval from max rate
    sendInterval = std::chrono::milliseconds(1000 / maxMessagesPerSecond);
    
    // Print startup message for diagnostic purposes
    std::cout << "Initializing UDP Telemetry Server on port " << port << std::endl;
}

// Destructor
UDPTelemetryServer::~UDPTelemetryServer() {
    stop();
}

// Start the server
bool UDPTelemetryServer::start() {
    if (running) {
        return true;  // Already running
    }
    
    if (!initializeSocket()) {
        return false;
    }
    
    std::cout << "UDP Telemetry Server started on port " << port << std::endl;
    std::cout << "Max message rate: " << maxMessagesPerSecond << " msgs/sec" << std::endl;
    std::cout << "Send interval: " << sendInterval.count() << " ms" << std::endl;
    
    running = true;
    sendThread = std::thread(&UDPTelemetryServer::senderLoop, this);
    
    return true;
}

// Stop the server
void UDPTelemetryServer::stop() {
    if (!running) {
        return;
    }
    
    running = false;
    queueCondition.notify_all();
    
    if (sendThread.joinable()) {
        sendThread.join();
    }
    
    closeSocket();
    
    std::cout << "UDP Telemetry Server stopped. Stats:" << std::endl;
    std::cout << "  Messages sent: " << messagesSent << std::endl;
    std::cout << "  Bytes sent: " << bytesSent << std::endl;
}

// Queue a message for sending
bool UDPTelemetryServer::queueMessage(TelemetryMessageType type, const std::vector<uint8_t>& payload) {
    if (!running) {
        return false;
    }
    
    std::unique_lock<std::mutex> lock(queueMutex);
    
    // Check queue size
    if (messageQueue.size() >= maxQueueSize) {
        // Queue is full, drop oldest message
        messageQueue.pop();
    }
    
    // Create new message
    TelemetryMessage message;
    message.messageType = type;
    message.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    message.sequenceNumber = sequenceCounter++;
    message.payload = payload;
    
    // Add to queue
    messageQueue.push(message);
    
    // Notify sender thread
    lock.unlock();
    queueCondition.notify_one();
    
    return true;
}

// Helper to queue control result (commented out for now)
/*
bool UDPTelemetryServer::queueControlResult(const ControlResult& result) {
    try {
        // Convert to JSON for easy serialization
        json j;
        
        // Add basic fields
        j["frameID"] = result.frameID;
        j["processingTimeMs"] = result.processingTimeMs;
        
        // Add control command
        j["command"]["u_x"] = result.command.u.x();
        j["command"]["u_y"] = result.command.u.y();
        j["command"]["u_z"] = result.command.u.z();
        j["command"]["u_dot_x"] = result.command.u_dot.x();
        j["command"]["u_dot_y"] = result.command.u_dot.y();
        j["command"]["u_dot_z"] = result.command.u_dot.z();
        j["command"]["timestamp"] = result.command.timestamp;
        
        // Handle performance data - support both original format and new generic format
        if (result.performanceData) {
            try {
                // First try to get the direct JSON representation (original format)
                std::string jsonStr = result.performanceData->toJson();
                j["performanceData"] = json::parse(jsonStr);
                j["controlType"] = result.performanceData->getControlType();
                
                // Also get the generic telemetry data format
                std::map<std::string, std::vector<std::string>> dataLabels;
                std::map<std::string, std::vector<double>> dataValues;
                result.performanceData->getTelemetryData(dataLabels, dataValues);
                
                // Only add these fields if there's data
                if (!dataLabels.empty()) {
                    j["dataLabels"] = dataLabels;
                    j["data"] = dataValues;
                }
            }
            catch (const json::parse_error& e) {
                std::cerr << "Error parsing performance data JSON: " << e.what() << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Error processing performance data: " << e.what() << std::endl;
            }
        }
        
        // Serialize to binary
        std::string jsonStr = j.dump();
        std::vector<uint8_t> payload(jsonStr.begin(), jsonStr.end());
        
        // Queue the message
        return queueMessage(TELEMETRY_CONTROL, payload);
    }
    catch (const std::exception& e) {
        std::cerr << "Error serializing control result: " << e.what() << std::endl;
        return false;
    }
}

// Helper to queue vision result
bool UDPTelemetryServer::queueVisionResult(const PipelineResult& result) {
    try {
        // Convert to JSON for serialization
        json j;
        
        // Add basic fields
        j["frameID"] = result.frameID;
        j["processingTimeMs"] = result.processingTimeMs;
        j["fps"] = result.fps;
        
        // Add critical data if available
        if (result.criticalData) {
            j["criticalData"]["type"] = result.criticalData->getDataType();
            
            // Special handling for ArUco critical data (if applicable)
            // This section would need to be adapted based on your actual ArUco data structure
            // Just included as a placeholder
        }
        
        // Add debug data if available
        if (result.debugData) {
            j["debugData"] = json::parse(result.debugData->toJson());
        }
        
        // Note: We don't send the actual images over UDP as they would be too large
        // Just indicate presence
        j["hasOutputFrame"] = !result.outputFrame.empty();
        j["debugImagesCount"] = result.debugImages.size();
        
        // Serialize to binary
        std::string jsonStr = j.dump();
        std::vector<uint8_t> payload(jsonStr.begin(), jsonStr.end());
        
        // Queue the message
        return queueMessage(TELEMETRY_VISION, payload);
    }
    catch (const std::exception& e) {
        std::cerr << "Error serializing vision result: " << e.what() << std::endl;
        return false;
    }
}
*/

// Helper to queue system status
bool UDPTelemetryServer::queueSystemStatus(const std::string& status) {
    try {
        // Create simple status JSON
        json j;
        j["status"] = status;
        j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        // Serialize to binary
        std::string jsonStr = j.dump();
        std::vector<uint8_t> payload(jsonStr.begin(), jsonStr.end());
        
        // Queue the message
        return queueMessage(TELEMETRY_SYSTEM, payload);
    }
    catch (const std::exception& e) {
        std::cerr << "Error serializing system status: " << e.what() << std::endl;
        return false;
    }
}

// Helper to queue generic telemetry data
bool UDPTelemetryServer::queueGenericTelemetry(const json& data) {
    try {
        // Serialize to binary
        std::string jsonStr = data.dump();
        std::vector<uint8_t> payload(jsonStr.begin(), jsonStr.end());
        
        // Queue the message using TELEMETRY_SYSTEM type (or add TELEMETRY_GENERIC)
        return queueMessage(TELEMETRY_SYSTEM, payload);
    }
    catch (const std::exception& e) {
        std::cerr << "Error serializing generic telemetry: " << e.what() << std::endl;
        return false;
    }
}

// Get port
uint16_t UDPTelemetryServer::getPort() const {
    return port;
}

// Get statistics
uint64_t UDPTelemetryServer::getMessagesSent() const {
    return messagesSent;
}

uint64_t UDPTelemetryServer::getBytesSent() const {
    return bytesSent;
}

// UDPControlTelemetryHandler implementation (commented out for now)
/*
UDPControlTelemetryHandler::UDPControlTelemetryHandler(
    std::shared_ptr<UDPTelemetryServer> server,
    bool sendAll,
    int sendEveryN)
    : telemetryServer(server),
      sendAllFrames(sendAll),
      sendEveryNFrames(sendEveryN),
      frameCounter(0) {}

void UDPControlTelemetryHandler::handleResult(const ControlResult& result) {
    frameCounter++;
    
    // Send all frames or every Nth frame
    if (sendAllFrames || (frameCounter % sendEveryNFrames == 0)) {
        telemetryServer->queueControlResult(result);
    }
}

void UDPControlTelemetryHandler::setSendRate(bool sendAll, int sendEveryN) {
    sendAllFrames = sendAll;
    sendEveryNFrames = sendEveryN;
}
*/
