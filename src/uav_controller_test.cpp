#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "UAVController.h"
#include "ControlTypes.h"
#include "Logger.h"

#include <sys/stat.h>
#include <signal.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#include <sys/types.h>
#define MKDIR(dir) mkdir(dir, 0755)
#endif

// Global controller pointer for signal handling
UAVController* g_controller = nullptr;

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;
    if (g_controller) {
        g_controller->stop();
    }
}

// Global function to create logs directory if it doesn't exist
void ensureLogDirectoryExists() {
    std::string dirPath = "logs";  
    struct stat info;
    
    // Check if directory exists and create it if it doesn't
    if (stat(dirPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        MKDIR(dirPath.c_str());
    }
}

// Print help message
void printHelp() {
    std::cout << "\nUAV Control Test Commands:\n"
              << "----------------------------\n"
              << "h, help     - Show this help message\n"
              << "s, status   - Show system status\n"
              << "v, viz      - Toggle HTTP visualization\n"
              << "p, port     - Show visualization port\n"
              << "t, takeoff  - Take off to 1.0m altitude (placeholder)\n"
              << "l, land     - Land the UAV (placeholder)\n"
              << "e, emergency- Emergency stop (placeholder)\n"
              << "q, quit     - Quit the program\n"
              << "----------------------------\n"
              << "Note: This version runs headless by default.\n"
              << "Enable HTTP visualization to monitor remotely.\n";
}

int main(int argc, char** argv) {
    // Parse command line arguments
    bool enableHttpViz = false;
    int httpPort = 8888;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--http" || arg == "-h") {
            enableHttpViz = true;
        } else if (arg == "--port" || arg == "-p") {
            if (i + 1 < argc) {
                httpPort = std::atoi(argv[++i]);
            }
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --http, -h        Enable HTTP visualization\n"
                      << "  --port PORT, -p   Set HTTP port (default: 8888)\n"
                      << "  --help            Show this help\n";
            return 0;
        }
    }
    
    // Setup signal handlers for graceful shutdown
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // Termination request
    
    // Ensure logs directory exists
    ensureLogDirectoryExists();
    
    // Initialize controller
    UAVController controller;
    g_controller = &controller;  // Set global pointer for signal handler
    
    // Configure HTTP visualization
    if (enableHttpViz) {
        controller.setHttpVisualization(true, httpPort);
        std::cout << "HTTP visualization enabled on port " << httpPort << std::endl;
        std::cout << "Open http://localhost:" << httpPort << " in your browser to monitor the system" << std::endl;
    } else {
        std::cout << "Running in headless mode (no visualization)" << std::endl;
        std::cout << "Use --http to enable web-based visualization" << std::endl;
    }
    
    std::cout << "\nInitializing UAV controller..." << std::endl;
    if (!controller.initialize()) {
        std::cerr << "Failed to initialize controller" << std::endl;
        g_controller = nullptr;
        return 1;
    }
    
    std::cout << "Starting controller threads..." << std::endl;
    if (!controller.start()) {
        std::cerr << "Failed to start controller" << std::endl;
        g_controller = nullptr;
        return 1;
    }
    
    // Print startup information
    std::cout << "\n=== UAV Controller Started ===" << std::endl;
    std::cout << "Mode: " << (enableHttpViz ? "HTTP Visualization" : "Headless") << std::endl;
    if (enableHttpViz) {
        std::cout << "Web Interface: http://localhost:" << httpPort << std::endl;
    }
    std::cout << "================================" << std::endl;
    
    // Print help message
    printHelp();
    
    // Main loop
    std::string command;
    bool userRequestedExit = false;
    
    while (!userRequestedExit && controller.isRunning()) {
        std::cout << "\nEnter command (h for help): ";
        
        // Use getline with timeout check
        if (!std::getline(std::cin, command)) {
            // EOF or input error, probably Ctrl+C
            break;
        }
        
        if (command.empty()) {
            continue;
        }
        
        char cmd = std::tolower(command[0]);
        
        switch (cmd) {
            case 'h':
                printHelp();
                break;
                
            case 's':
                std::cout << "System Status:" << std::endl;
                std::cout << "  Running: " << (controller.isRunning() ? "Yes" : "No") << std::endl;
                std::cout << "  HTTP Visualization: " << (controller.getVisualizationPort() > 0 ? "Enabled" : "Disabled") << std::endl;
                if (controller.getVisualizationPort() > 0) {
                    std::cout << "  Visualization URL: http://localhost:" << controller.getVisualizationPort() << std::endl;
                }
                break;
                
            case 'v':
                if (controller.getVisualizationPort() > 0) {
                    std::cout << "HTTP visualization is enabled on port " << controller.getVisualizationPort() << std::endl;
                    std::cout << "Access it at: http://localhost:" << controller.getVisualizationPort() << std::endl;
                } else {
                    std::cout << "HTTP visualization is disabled. Restart with --http to enable." << std::endl;
                }
                break;
                
            case 'p':
                if (controller.getVisualizationPort() > 0) {
                    std::cout << "Visualization port: " << controller.getVisualizationPort() << std::endl;
                    std::cout << "URL: http://localhost:" << controller.getVisualizationPort() << std::endl;
                } else {
                    std::cout << "HTTP visualization is not enabled" << std::endl;
                }
                break;
                
            case 't':
                std::cout << "Takeoff command received (not implemented yet)" << std::endl;
                // TODO: Implement takeoff logic
                break;
                
            case 'l':
                std::cout << "Land command received (not implemented yet)" << std::endl;
                // TODO: Implement landing logic
                break;
                
            case 'e':
                std::cout << "Emergency stop command received (not implemented yet)" << std::endl;
                // TODO: Implement emergency stop logic
                break;
                
            case 'q':
                std::cout << "Quitting..." << std::endl;
                userRequestedExit = true;
                break;
                
            default:
                std::cout << "Unknown command '" << command << "'. Type 'h' for help." << std::endl;
                break;
        }
    }
    
    // Stop controller
    std::cout << "Stopping controller..." << std::endl;
    controller.stop();
    
    // Clear global pointer
    g_controller = nullptr;
    
    std::cout << "Done!" << std::endl;
    return 0;
}