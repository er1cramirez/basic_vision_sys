#!/bin/bash

# Build script for the telemetry system examples

echo "Building Generic Telemetry System..."

cd /home/eric/droneSim_ws/src/basic_vision_sys

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi

cd build

# Configure and build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build completed successfully!"
    echo ""
    echo "Available executables:"
    echo "  ./simple_telemetry_example  - Basic telemetry example"
    echo "  ./telemetry_example         - Advanced telemetry example"
    echo "  ./uav_controller_test       - Main controller with telemetry"
    echo ""
    echo "To view telemetry data:"
    echo "  1. Run one of the examples above"
    echo "  2. In another terminal: python3 ../tools/generic_telemetry_viewer.py"
    echo "  3. Open http://localhost:8050 in your browser"
    echo ""
else
    echo "❌ Build failed!"
    exit 1
fi
