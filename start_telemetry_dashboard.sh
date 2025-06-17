#!/bin/bash

# Script to start the telemetry dashboard
echo "Starting EKF Telemetry Dashboard..."
echo "This will open a web dashboard to visualize EKF state data"
echo ""

# Find an available port
for port in 8080 8081 8082 8083 8084; do
    if ! lsof -i:$port > /dev/null 2>&1; then
        echo "Using port $port for the dashboard"
        echo "Open http://localhost:$port in your browser"
        echo ""
        echo "To see telemetry data:"
        echo "1. Run your UAV controller: ./build/uav_controller_test"  
        echo "2. Or run the test: ./build/test_telemetry"
        echo ""
        
        # Start the dashboard
        python3 tools/generic_telemetry_viewer.py --web-port $port
        exit 0
    fi
done

echo "Could not find an available port!"
exit 1
