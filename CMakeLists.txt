cmake_minimum_required(VERSION 3.10)
project(VisionSystem VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)  # C++17 required for std::optional
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Option to enable/disable Gazebo support
option(WITH_GAZEBO "Build with Gazebo support" OFF)

# Define a preprocessor macro to indicate Gazebo availability
if(WITH_GAZEBO)
    # Setup environment for Gazebo Harmonic
    set(ENV{GZ_VERSION} "harmonic")
    
    # Find Gazebo Harmonic packages
    find_package(gz-transport13 REQUIRED)
    find_package(gz-msgs10 REQUIRED)
    
    add_definitions(-DWITH_GAZEBO)
endif()



set(MAVLINK_INCLUDE_DIR $ENV{HOME}/mavlink/generated/include/mavlink/eric)
# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mavcomm
    ${MAVLINK_INCLUDE_DIR}
)
add_compile_options(-Wno-address-of-packed-member)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vision/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vision/pipelines
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mavcomm
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
# Add Gazebo include directories only if enabled
if(WITH_GAZEBO)
    include_directories(
        ${gz-transport13_INCLUDE_DIRS}
        ${gz-msgs10_INCLUDE_DIRS}
    )
endif()

# Define source file groups

set(MAV_COM_SOURCES
    src/mavcomm/mavlink_comm_module.cpp
    src/mavcomm/serial_port.cpp
    src/mavcomm/udp_port.cpp
)
set(VISION_CORE_SOURCES
    src/vision/core/CameraSource.cpp
)
# Add Gazebo sources only if enabled
if(WITH_GAZEBO)
    list(APPEND VISION_CORE_SOURCES src/vision/core/GazeboSource.cpp)
    # list(APPEND VISION_CORE_HEADERS include/vision/core/GazeboSource.h)
endif()

# Add Logger source file
set(LOGGER_SOURCES
    src/common/Logger.cpp
)

set(PIPELINE_SOURCES
    src/vision/pipelines/aruco_pose_pipeline.cpp
    src/vision/pipelines/aruco_ekf_estimator.cpp
)

set(CONTROL_SOURCES
    src/core/UAVStateMachine.cpp
    src/control/UAVController.cpp
)

set(MAIN_SOURCES
    src/main.cpp
)

set(TEST_SOURCES
    src/aruco_pose_test.cpp
)

set(CONTROLLER_TEST_SOURCES
    src/uav_controller_test.cpp
)

# Create main executable
# add_executable(vision_system 
#     ${MAIN_SOURCES} 
#     ${CORE_SOURCES}
# )

# # Link libraries for main executable
# target_link_libraries(vision_system
#     ${OpenCV_LIBS}
#     ${gz-transport13_LIBRARIES}
#     ${gz-msgs10_LIBRARIES}
#     pthread
# )

# # Create ArUco test executable
# add_executable(aruco_pose_test 
#     ${TEST_SOURCES}
#     ${PIPELINE_SOURCES}
#     ${CORE_SOURCES}
#     ${LOGGER_SOURCES}
# )

# # Link libraries for ArUco test
# target_link_libraries(aruco_pose_test
#     ${OpenCV_LIBS}
#     ${gz-transport13_LIBRARIES}
#     ${gz-msgs10_LIBRARIES}
#     pthread
#     Eigen3::Eigen
# )

# Create UAV Controller test executable
add_executable(uav_controller_test 
    ${CONTROLLER_TEST_SOURCES}
    ${PIPELINE_SOURCES}
    ${VISION_CORE_SOURCES}
    ${LOGGER_SOURCES}
    ${CONTROL_SOURCES}
    ${MAV_COM_SOURCES}
)

# Link libraries for UAV Controller test
target_link_libraries(uav_controller_test
    ${OpenCV_LIBS}
    pthread
    Eigen3::Eigen
)
# Link Gazebo libraries only if enabled
if(WITH_GAZEBO)
    target_link_libraries(uav_controller_test
        ${gz-transport13_LIBRARIES}
        ${gz-msgs10_LIBRARIES}
    )
endif()

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

# Create logs directory if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/logs)