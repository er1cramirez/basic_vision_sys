// src/main.cpp
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "ImageSourceFactory.h"

// Function to process frames from any source
void processFrames(ImageSourcePtr source, const std::string& windowName, int maxFrames = 1000) {
    if (!source || !source->initialize()) {
        std::cerr << "Failed to initialize image source" << std::endl;
        return;
    }
    
    cv::Mat frame;
    int frameCount = 0;
    
    // For FPS calculation
    auto startTime = std::chrono::steady_clock::now();
    int fpsCounter = 0;
    double actualFps = 0;
    
    std::cout << "Processing frames from " << windowName << "..." << std::endl;
    
    while (frameCount < maxFrames) {
        if (source->getNextFrame(frame)) {
            if (!frame.empty()) {
                // Process the frame (add your computer vision operations here)
                
                // Display the frame
                cv::imshow(windowName, frame);
                
                frameCount++;
                fpsCounter++;
                
                // Calculate FPS every second
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - startTime).count();
                
                if (elapsed >= 1000) {
                    actualFps = fpsCounter * 1000.0 / elapsed;
                    std::cout << windowName << " FPS: " << actualFps 
                              << " (Target: " << source->getFPS() << ")" << std::endl;
                    
                    fpsCounter = 0;
                    startTime = now;
                }
                // Draw a cross in the center of the frame
                int centerX = frame.cols / 2;
                int centerY = frame.rows / 2;
                int lineLength = 20; // Length of each arm of the cross

                // Draw horizontal line
                cv::line(frame, 
                         cv::Point(centerX - lineLength, centerY),
                         cv::Point(centerX + lineLength, centerY),
                         cv::Scalar(0, 0, 255), // Red color (BGR)
                         2); // Thickness

                // Draw vertical line
                cv::line(frame, 
                         cv::Point(centerX, centerY - lineLength),
                         cv::Point(centerX, centerY + lineLength),
                         cv::Scalar(0, 0, 255), // Red color (BGR)
                         2); // Thickness
                // Add text overlay showing FPS
                cv::putText(frame, 
                           "FPS: " + std::to_string(static_cast<int>(actualFps)),
                           cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX,
                           1.0,
                           cv::Scalar(0, 255, 0),
                           2);
                
                // Show the frame
                cv::imshow(windowName, frame);
            }
        }
        
        // Check for key press (27 = ESC key)
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    
    // Release resources
    cv::destroyWindow(windowName);
    source->release();
}

int main(int argc, char** argv) {
    // Parse command line arguments
    bool useGazebo = false;
    std::string gazeboTopic = "/world/map/model/iris/link/camera_link/sensor/camera/image";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--gazebo" || arg == "-g") {
            useGazebo = true;
        } else if (arg == "--topic" || arg == "-t") {
            if (i + 1 < argc) {
                gazeboTopic = argv[i + 1];
                i++;
            }
        }
    }
    
    // Create the appropriate image source
    ImageSourcePtr source;
    std::string windowName;
    
    if (useGazebo) {
        std::cout << "Using Gazebo source with topic: " << gazeboTopic << std::endl;
        source = ImageSourceFactory::createGazeboSource(gazeboTopic);
        windowName = "Gazebo Camera";
    } else {
        std::cout << "Using physical camera" << std::endl;
        source = ImageSourceFactory::createCameraSource();
        windowName = "Physical Camera";
    }
    
    // Process frames from the selected source
    processFrames(source, windowName);
    
    return 0;
}
