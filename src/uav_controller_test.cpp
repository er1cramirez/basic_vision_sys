#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "UAVController.h"
#include "ControlTypes.h"
#include "Logger.h"

#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#include <sys/types.h>
#define MKDIR(dir) mkdir(dir, 0755)
#endif

// Global function to create logs directory if it doesn't exist
void ensureLogDirectoryExists() {
    std::string dirPath = "~/DroneSimLogs";
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
              << "q, quit     - Quit the program\n"
              << "----------------------------\n";
}

int main(int argc, char** argv) {
    // Ensure logs directory exists
    ensureLogDirectoryExists();
    
    // Parse command line arguments
    bool useGazebo = true;    // Default to Gazebo for testing
    std::string gazeboTopic = "/world/iris_runway_aruco/model/iris_with_fixed_camera/model/camera/link/camera_link/sensor/camera/image";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--camera" || arg == "-c") {
            useGazebo = false;
        } else if (arg == "--topic" || arg == "-t") {
            if (i + 1 < argc) {
                gazeboTopic = argv[i + 1];
                i++;
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "UAV Control Test\n\n"
                      << "Options:\n"
                      << "  -c, --camera       Use physical camera instead of Gazebo\n"
                      << "  -t, --topic TOPIC  Set Gazebo camera topic\n"
                      << "  -h, --help         Show this help message\n"
                      << std::endl;
            return 0;
        }
    }
    
    // Initialize controller
    UAVController controller;
    
    std::cout << "Initializing UAV controller..." << std::endl;
    if (!controller.initialize(gazeboTopic, !useGazebo)) {
        std::cerr << "Failed to initialize controller" << std::endl;
        return 1;
    }
     
    std::cout << "Starting controller threads..." << std::endl;
    if (!controller.start()) {
        std::cerr << "Failed to start controller" << std::endl;
        return 1;
    }
    
    // Print help message
    printHelp();
    
    // Main loop
    std::string command;
    bool running = true;
    
    while (running && controller.isRunning()) {
        std::cout << "\nEnter command (h for help): ";
        std::getline(std::cin, command);
        
        if (command.empty()) {
            continue;
        }
        
        char cmd = command[0];
        
        switch (cmd) {
            case 'h':
            case 'H':
                printHelp();
                break;
                  
            case 'v':
            case 'V':
                std::cout << "Setting velocity to (0.2, 0, 0)..." << std::endl;
                controller.setVelocity(Eigen::Vector3d(0.2, 0.0, 0.0));
                break;
                
            case 'q':
            case 'Q':
                std::cout << "Quitting..." << std::endl;
                running = false;
                break;
                
            default:
                std::cout << "Unknown command. Type 'h' for help." << std::endl;
                break;
        }
    }
    
    // Stop controller
    std::cout << "Stopping controller..." << std::endl;
    controller.stop();
    
    std::cout << "Done!" << std::endl;
    return 0;
}