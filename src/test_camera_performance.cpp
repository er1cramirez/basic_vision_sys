// test_camera_performance.cpp
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "CameraSource.h"

void printStats(const std::string& name, const std::vector<double>& times) {
    if (times.empty()) return;
    
    double sum = std::accumulate(times.begin(), times.end(), 0.0);
    double mean = sum / times.size();
    
    auto minmax = std::minmax_element(times.begin(), times.end());
    
    std::cout << name << ":" << std::endl;
    std::cout << "  Mean: " << std::fixed << std::setprecision(2) << mean << " ms" << std::endl;
    std::cout << "  Min:  " << std::fixed << std::setprecision(2) << *minmax.first << " ms" << std::endl;
    std::cout << "  Max:  " << std::fixed << std::setprecision(2) << *minmax.second << " ms" << std::endl;
    std::cout << "  FPS:  " << std::fixed << std::setprecision(1) << (1000.0 / mean) << std::endl;
}

int main(int argc, char** argv) {
    int cameraId = 0;
    int width = 640;
    int height = 480;
    double fps = 30.0;
    int numFrames = 100;
    
    // Parse arguments
    if (argc > 1) cameraId = std::atoi(argv[1]);
    if (argc > 2) width = std::atoi(argv[2]);
    if (argc > 3) height = std::atoi(argv[3]);
    if (argc > 4) fps = std::atof(argv[4]);
    if (argc > 5) numFrames = std::atoi(argv[5]);
    
    std::cout << "Camera Performance Test" << std::endl;
    std::cout << "======================" << std::endl;
    std::cout << "Camera ID: " << cameraId << std::endl;
    std::cout << "Resolution: " << width << "x" << height << std::endl;
    std::cout << "Target FPS: " << fps << std::endl;
    std::cout << "Test frames: " << numFrames << std::endl;
    std::cout << std::endl;
    
    // Test 1: Direct OpenCV VideoCapture
    std::cout << "Test 1: Direct OpenCV VideoCapture" << std::endl;
    {
        cv::VideoCapture cap(cameraId, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap.set(cv::CAP_PROP_FPS, fps);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        
        if (!cap.isOpened()) {
            std::cerr << "Failed to open camera!" << std::endl;
            return 1;
        }
        
        // Warm up
        cv::Mat frame;
        for (int i = 0; i < 10; i++) {
            cap.read(frame);
        }
        
        std::vector<double> readTimes;
        std::vector<double> grabTimes;
        std::vector<double> retrieveTimes;
        
        // Test read()
        for (int i = 0; i < numFrames; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            bool success = cap.read(frame);
            auto end = std::chrono::high_resolution_clock::now();
            
            if (success && !frame.empty()) {
                double ms = std::chrono::duration<double, std::milli>(end - start).count();
                readTimes.push_back(ms);
            }
        }
        
        // Test grab() + retrieve()
        for (int i = 0; i < numFrames; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            bool success = cap.grab();
            auto grabEnd = std::chrono::high_resolution_clock::now();
            
            if (success) {
                success = cap.retrieve(frame);
                auto end = std::chrono::high_resolution_clock::now();
                
                if (success && !frame.empty()) {
                    double grabMs = std::chrono::duration<double, std::milli>(grabEnd - start).count();
                    double retrieveMs = std::chrono::duration<double, std::milli>(end - grabEnd).count();
                    grabTimes.push_back(grabMs);
                    retrieveTimes.push_back(retrieveMs);
                }
            }
        }
        
        printStats("  read()", readTimes);
        printStats("  grab()", grabTimes);
        printStats("  retrieve()", retrieveTimes);
        
        cap.release();
    }
    
    std::cout << std::endl;
    
    // Test 2: Optimized CameraSource
    std::cout << "Test 2: Optimized CameraSource" << std::endl;
    {
        CameraSource camera(cameraId, width, height, fps);
        
        if (!camera.initialize()) {
            std::cerr << "Failed to initialize CameraSource!" << std::endl;
            return 1;
        }
        
        camera.printCameraModes();
        
        // Test with copy
        camera.setShouldCopyFrame(true);
        std::vector<double> copyTimes;
        cv::Mat frame;
        
        for (int i = 0; i < numFrames; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            bool success = camera.getNextFrame(frame);
            auto end = std::chrono::high_resolution_clock::now();
            
            if (success) {
                double ms = std::chrono::duration<double, std::milli>(end - start).count();
                copyTimes.push_back(ms);
            }
        }
        
        // Test without copy
        camera.setShouldCopyFrame(false);
        std::vector<double> noCopyTimes;
        
        for (int i = 0; i < numFrames; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            bool success = camera.getNextFrame(frame);
            auto end = std::chrono::high_resolution_clock::now();
            
            if (success) {
                double ms = std::chrono::duration<double, std::milli>(end - start).count();
                noCopyTimes.push_back(ms);
            }
        }
        
        printStats("  With copy", copyTimes);
        printStats("  Without copy", noCopyTimes);
        
        camera.release();
    }
    
    std::cout << std::endl;
    
    // Test 3: Different buffer sizes
    std::cout << "Test 3: Buffer Size Impact" << std::endl;
    {
        for (int bufSize : {1, 2, 4}) {
            cv::VideoCapture cap(cameraId, cv::CAP_V4L2);
            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
            cap.set(cv::CAP_PROP_FPS, fps);
            cap.set(cv::CAP_PROP_BUFFERSIZE, bufSize);
            
            if (!cap.isOpened()) continue;
            
            // Warm up
            cv::Mat frame;
            for (int i = 0; i < 10; i++) {
                cap.read(frame);
            }
            
            std::vector<double> times;
            for (int i = 0; i < numFrames; i++) {
                auto start = std::chrono::high_resolution_clock::now();
                bool success = cap.read(frame);
                auto end = std::chrono::high_resolution_clock::now();
                
                if (success && !frame.empty()) {
                    double ms = std::chrono::duration<double, std::milli>(end - start).count();
                    times.push_back(ms);
                }
            }
            
            printStats("  Buffer size " + std::to_string(bufSize), times);
            cap.release();
        }
    }
    
    return 0;
}