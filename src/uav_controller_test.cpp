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
              << "t, takeoff  - Take off to 1.0m altitude\n"
              << "l, land     - Land the UAV\n"
              << "e, emergency- Emergency stop\n"
              << "p, position - Go to position (0, 0, 1.5)\n"
              << "w, waypoints- Follow a set of demo waypoints\n"
              << "v, velocity - Set a demo velocity (0.2, 0, 0)\n"
              << "q, quit     - Quit the program\n"
              << "----------------------------\n";
}

int main(int argc, char** argv) {
    // Ensure logs directory exists
    ensureLogDirectoryExists();
    
    // Initialize controller
    UAVController controller;
    
    std::cout << "Initializing UAV controller..." << std::endl;
    if (!controller.initialize()) {
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