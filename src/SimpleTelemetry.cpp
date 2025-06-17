#include "SimpleTelemetry.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// Static member initialization
std::shared_ptr<SimpleTelemetryManager> SimpleTelemetryManager::instance = nullptr;
std::mutex SimpleTelemetryManager::instanceMutex;

SimpleTelemetryManager::~SimpleTelemetryManager() {
    shutdown();
}

std::shared_ptr<SimpleTelemetryManager> SimpleTelemetryManager::getInstance() {
    std::lock_guard<std::mutex> lock(instanceMutex);
    if (!instance) {
        instance = std::shared_ptr<SimpleTelemetryManager>(new SimpleTelemetryManager());
    }
    return instance;
}

bool SimpleTelemetryManager::initialize(uint16_t port, size_t maxHistory) {
    maxHistorySize = maxHistory;
    
    if (!initUDP(port)) {
        std::cerr << "Failed to initialize UDP socket" << std::endl;
        return false;
    }
    
    running = true;
    publishThread = std::thread(&SimpleTelemetryManager::publishLoop, this);
    
    std::cout << "Simple telemetry initialized on port " << port << std::endl;
    return true;
}

void SimpleTelemetryManager::shutdown() {
    if (running) {
        running = false;
        if (publishThread.joinable()) {
            publishThread.join();
        }
        if (sockfd >= 0) {
            close(sockfd);
        }
    }
}

bool SimpleTelemetryManager::initUDP(uint16_t port) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        return false;
    }
    
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_port = htons(port);
    
    return true;
}

void SimpleTelemetryManager::logNumber(const std::string& name, double value) {
    std::lock_guard<std::mutex> lock(dataMutex);
    data[name].emplace_back(value);
    
    // Maintain history limit
    if (data[name].size() > maxHistorySize) {
        data[name].pop_front();
    }
}

void SimpleTelemetryManager::logVector(const std::string& name, double x, double y, double z) {
    std::vector<double> vec = {x, y, z};
    std::lock_guard<std::mutex> lock(dataMutex);
    data[name].emplace_back(vec);
    
    if (data[name].size() > maxHistorySize) {
        data[name].pop_front();
    }
}

void SimpleTelemetryManager::logString(const std::string& name, const std::string& value) {
    std::lock_guard<std::mutex> lock(dataMutex);
    data[name].emplace_back(value);
    
    if (data[name].size() > maxHistorySize) {
        data[name].pop_front();
    }
}

void SimpleTelemetryManager::logBool(const std::string& name, bool value) {
    std::lock_guard<std::mutex> lock(dataMutex);
    data[name].emplace_back(value);
    
    if (data[name].size() > maxHistorySize) {
        data[name].pop_front();
    }
}

std::map<std::string, std::vector<SimpleTelemetryPoint>> SimpleTelemetryManager::getAllData() const {
    std::lock_guard<std::mutex> lock(dataMutex);
    
    std::map<std::string, std::vector<SimpleTelemetryPoint>> result;
    for (const auto& entry : data) {
        result[entry.first] = std::vector<SimpleTelemetryPoint>(entry.second.begin(), entry.second.end());
    }
    return result;
}

void SimpleTelemetryManager::publishLoop() {
    while (running) {
        try {
            json telemetryData;
            telemetryData["type"] = "telemetry_update";
            telemetryData["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            json entriesJson = json::object();
            
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                for (const auto& entry : data) {
                    if (!entry.second.empty()) {
                        const auto& latest = entry.second.back();
                        
                        json entryJson;
                        entryJson["name"] = entry.first;
                        
                        // Convert value to JSON
                        std::visit([&entryJson](const auto& value) {
                            entryJson["latestValue"] = value;
                        }, latest.value);
                        
                        auto timeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                            latest.timestamp.time_since_epoch()).count();
                        entryJson["latestTimestamp"] = timeMs;
                        
                        entriesJson[entry.first] = entryJson;
                    }
                }
            }
            
            telemetryData["entries"] = entriesJson;
            telemetryData["entryCount"] = entriesJson.size();
            
            // Create UDP packet
            std::string jsonStr = telemetryData.dump();
            
            // Simple packet format: [type(4)][timestamp(8)][seq(4)][size(4)][payload]
            std::vector<uint8_t> packet;
            
            uint32_t msgType = htonl(3); // TELEMETRY_SYSTEM
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&msgType), 
                         reinterpret_cast<uint8_t*>(&msgType) + 4);
            
            uint64_t timestamp = htobe64(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&timestamp), 
                         reinterpret_cast<uint8_t*>(&timestamp) + 8);
            
            static uint32_t seq = 0;
            uint32_t seqNum = htonl(seq++);
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&seqNum), 
                         reinterpret_cast<uint8_t*>(&seqNum) + 4);
            
            uint32_t payloadSize = htonl(static_cast<uint32_t>(jsonStr.size()));
            packet.insert(packet.end(), reinterpret_cast<uint8_t*>(&payloadSize), 
                         reinterpret_cast<uint8_t*>(&payloadSize) + 4);
            
            packet.insert(packet.end(), jsonStr.begin(), jsonStr.end());
            
            // Send packet
            sendto(sockfd, packet.data(), packet.size(), 0, 
                   reinterpret_cast<struct sockaddr*>(&serverAddr), sizeof(serverAddr));
            
        } catch (const std::exception& e) {
            std::cerr << "Error in publish loop: " << e.what() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }
}
