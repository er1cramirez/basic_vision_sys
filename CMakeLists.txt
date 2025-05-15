cmake_minimum_required(VERSION 3.10)
project(VisionSystem VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)

# Setup environment for Gazebo Harmonic
set(ENV{GZ_VERSION} "harmonic")

# Find Gazebo Harmonic packages
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vision/core
    ${OpenCV_INCLUDE_DIRS}
    ${gz-transport13_INCLUDE_DIRS}
    ${gz-msgs10_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/main.cpp
    src/vision/core/CameraSource.cpp
    src/vision/core/GazeboSource.cpp
)

# Create executable
add_executable(vision_system ${SOURCES})

# Link libraries
target_link_libraries(vision_system
    ${OpenCV_LIBS}
    ${gz-transport13_LIBRARIES}
    ${gz-msgs10_LIBRARIES}
    pthread
)
