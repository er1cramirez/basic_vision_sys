cmake_minimum_required(VERSION 3.10)
project(VisionSystem VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)  # C++17 required for std::optional
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)  # Added Eigen3 requirement

# Setup environment for Gazebo Harmonic
set(ENV{GZ_VERSION} "harmonic")

# Find Gazebo Harmonic packages
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vision/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vision/pipelines
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}    # Added Eigen include
    ${gz-transport13_INCLUDE_DIRS}
    ${gz-msgs10_INCLUDE_DIRS}
)

# Define source file groups
set(CORE_SOURCES
    src/vision/core/CameraSource.cpp
    src/vision/core/GazeboSource.cpp
)

set(PIPELINE_SOURCES
    src/vision/pipelines/aruco_pose_pipeline.cpp
)

set(MAIN_SOURCES
    src/main.cpp
)

set(TEST_SOURCES
    src/aruco_pose_test.cpp
)

# Create main executable
add_executable(vision_system 
    ${MAIN_SOURCES} 
    ${CORE_SOURCES}
)

# Link libraries for main executable
target_link_libraries(vision_system
    ${OpenCV_LIBS}
    ${gz-transport13_LIBRARIES}
    ${gz-msgs10_LIBRARIES}
    pthread
)

# Create ArUco test executable - include ALL required source files
add_executable(aruco_pose_test 
    ${TEST_SOURCES}
    ${PIPELINE_SOURCES}
    ${CORE_SOURCES}  # Add core sources which include CameraSource and GazeboSource
)

# Link libraries for ArUco test
target_link_libraries(aruco_pose_test
    ${OpenCV_LIBS}
    ${gz-transport13_LIBRARIES}
    ${gz-msgs10_LIBRARIES}
    pthread
    Eigen3::Eigen  # Link Eigen3
)

# Add build-specific compiler optimizations for release builds
if(CMAKE_BUILD_TYPE MATCHES "Release")
    # Enable highest optimization level
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
    
    # Check for SIMD support and enable if available
    include(CheckCXXCompilerFlag)
    
    # Check for AVX2
    check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
    if(COMPILER_SUPPORTS_AVX2)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mavx2")
        message(STATUS "Enabling AVX2 instructions")
    endif()
    
    # Check for SSE4.1 as fallback
    check_cxx_compiler_flag("-msse4.1" COMPILER_SUPPORTS_SSE41)
    if(COMPILER_SUPPORTS_SSE41 AND NOT COMPILER_SUPPORTS_AVX2)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1")
        message(STATUS "Enabling SSE4.1 instructions")
    endif()
    
    # Enable link-time optimization
    check_cxx_compiler_flag("-flto" COMPILER_SUPPORTS_LTO)
    if(COMPILER_SUPPORTS_LTO)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
        message(STATUS "Enabling link-time optimization")
    endif()
endif()