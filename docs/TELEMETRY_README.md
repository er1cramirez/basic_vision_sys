# Generic Telemetry System

A WPILib SmartDashboard-inspired telemetry system for real-time data visualization and logging. This system allows you to add telemetry data asynchronously from anywhere in your code without being tied to specific data types or structures.

## Features

- **Generic Data Types**: Supports scalars, vectors, strings, arrays, and custom types
- **Non-invasive**: Minimal impact on performance, fire-and-forget logging
- **Real-time Visualization**: Live web-based dashboard with interactive graphs
- **Automatic Discovery**: Dynamically discovers and organizes telemetry entries
- **Thread-safe**: Safe to use from multiple threads simultaneously
- **WPILib-style API**: Familiar interface if you've used FIRST Robotics tools

## Quick Start

### 1. Include the Header
```cpp
#include "Telemetry.h"  // Just include this one header!
```

### 2. Log Data
```cpp
// Simple scalar values
TEL_NUMBER("control/position_x", currentPosition.x);
TEL_NUMBER("control/velocity", currentVelocity);
TEL_BOOL("system/is_enabled", isSystemEnabled);

// Vectors
TEL_VECTOR("control/position", pos.x, pos.y, pos.z);

// Events and status
TEL_EVENT("System initialized");
TEL_INFO("Tracking target successfully");
TEL_WARNING("High error detected");

// Automatic timing
TEL_FUNCTION_TIMER();  // Times the entire function
{
    TEL_AUTO_TIMER("expensive_operation");
    // ... do work ...
}  // Automatically logs timing when scope exits
```

### 3. View Live Data
Run the Python visualizer:
```bash
python3 tools/generic_telemetry_viewer.py
```
Open http://localhost:8050 in your browser.

## API Reference

### Basic Logging Functions

| Function | Description | Example |
|----------|-------------|---------|
| `TEL_NUMBER(name, value)` | Log a numeric value | `TEL_NUMBER("speed", 5.2)` |
| `TEL_VECTOR(name, x, y, z)` | Log a 3D vector | `TEL_VECTOR("position", 1, 2, 3)` |
| `TEL_BOOL(name, value)` | Log a boolean | `TEL_BOOL("enabled", true)` |
| `TEL_STRING(name, value)` | Log a string | `TEL_STRING("status", "OK")` |
| `TEL_ARRAY(name, vector)` | Log a numeric array | `TEL_ARRAY("data", myVector)` |

### Event Logging

| Function | Description |
|----------|-------------|
| `TEL_EVENT(message)` | Log a general event |
| `TEL_INFO(message)` | Log an info message |
| `TEL_WARNING(message)` | Log a warning |
| `TEL_ERROR(message)` | Log an error |

### Timing Functions

| Function | Description |
|----------|-------------|
| `TEL_TIMER_START(name)` | Start a named timer |
| `TEL_TIMER_END(name)` | End a named timer |
| `TEL_AUTO_TIMER(name)` | RAII timer - ends automatically |
| `TEL_FUNCTION_TIMER()` | Times the current function |

### Performance Monitoring

| Function | Description |
|----------|-------------|
| `TEL_FPS(component, fps)` | Record FPS for a component |
| `TEL_FRAME_TIME(component, ms)` | Record frame processing time |

### Helper Functions

The `TelemetryHelpers` namespace provides common patterns:

```cpp
// Log position with automatic derived values
TelH::logPosition("control", x, y, z);
// Creates: control/position, control/position_x, control/position_y, 
//          control/position_z, control/position_magnitude

// Log tracking error
TelH::logError("control", ref_x, ref_y, ref_z, actual_x, actual_y, actual_z);

// Log system health
TelH::logHealth("vision", isHealthy, "Tracking well");

// Log performance metrics
TelH::logPerformance("control", fps, avgFrameTime, maxFrameTime);
```

## Integration Examples

### Control System Integration
```cpp
class MyController {
public:
    void update() {
        TEL_FUNCTION_TIMER();
        
        // Calculate control
        Vector3d error = reference - currentPosition;
        Vector3d control = pidController.update(error);
        
        // Log everything
        TEL_VECTOR("control/reference", reference.x(), reference.y(), reference.z());
        TEL_VECTOR("control/position", currentPosition.x(), currentPosition.y(), currentPosition.z());
        TEL_VECTOR("control/error", error.x(), error.y(), error.z());
        TEL_VECTOR("control/output", control.x(), control.y(), control.z());
        TEL_NUMBER("control/error_magnitude", error.norm());
        
        // Log system status
        bool isStable = error.norm() < 0.1;
        TEL_BOOL("control/is_stable", isStable);
        
        if (!isStable) {
            TEL_WARNING("High tracking error: " + std::to_string(error.norm()));
        }
    }
};
```

### Vision System Integration
```cpp
class VisionPipeline {
public:
    void processFrame() {
        TEL_AUTO_TIMER("vision_processing");
        
        // Process frame
        bool detected = detectTarget();
        
        // Log results
        TEL_BOOL("vision/target_detected", detected);
        TEL_FPS("vision", getCurrentFPS());
        
        if (detected) {
            TEL_VECTOR("vision/target_position", targetPos.x, targetPos.y, targetPos.z);
            TEL_NUMBER("vision/confidence", detectionConfidence);
            TEL_INFO("Target detected at: [" + std::to_string(targetPos.x) + ", " + 
                     std::to_string(targetPos.y) + ", " + std::to_string(targetPos.z) + "]");
        }
    }
};
```

## Advanced Usage

### Custom Telemetry Entries
For more control, you can use the underlying publisher directly:

```cpp
auto publisher = Telemetry::getPublisher();

// Create custom entries with specific categories and units
auto positionEntry = publisher->getVectorEntry("robot/position", "state", "meters");
auto velocityEntry = publisher->getVectorEntry("robot/velocity", "state", "m/s");

// Use them
positionEntry->setValue(x, y, z);
velocityEntry->setValue(vx, vy, vz);
```

### Configuration
```cpp
// Initialize with custom settings
TelemetryInit::initialize(14559, 30);  // Port 14559, 30 Hz update rate

// Check if initialized
if (TelemetryInit::isInitialized()) {
    TEL_INFO("Telemetry active");
}

// Shutdown when done
TelemetryInit::shutdown();
```

## Data Organization

The system automatically organizes data into categories:

- `control/` - Control system data
- `vision/` - Vision system data  
- `performance/` - Performance metrics
- `timers/` - Timing data
- `health/` - System health status
- `events/` - Event logs

You can create custom categories by using `/` in entry names:
```cpp
TEL_NUMBER("sensors/imu/accel_x", accelX);
TEL_NUMBER("navigation/waypoint_distance", distance);
```

## Python Visualizer

The Python visualizer provides:

- **Live Time Series Plots**: Interactive graphs of numeric data
- **Current Values Display**: Real-time dashboard of all scalar values
- **Event Log**: Chronological list of events and messages
- **Category Organization**: Data grouped by category for easy navigation
- **Export Functionality**: Save data for offline analysis

### Running the Visualizer
```bash
# Basic usage
python3 tools/generic_telemetry_viewer.py

# Custom ports
python3 tools/generic_telemetry_viewer.py --telemetry-port 14559 --web-port 8050

# Quiet mode
python3 tools/generic_telemetry_viewer.py --quiet
```

## Building

The telemetry system is integrated into the main CMake build:

```bash
mkdir build && cd build
cmake ..
make

# Run examples
./telemetry_example
./simple_telemetry_example
```

## Dependencies

- C++17 compiler
- nlohmann/json (header-only)
- Threads
- Python 3.7+ (for visualizer)
- dash, plotly (Python packages)

## Performance Notes

- Telemetry calls are designed to be very fast (typically < 1μs)
- Data is queued asynchronously and sent in batches
- Memory usage is bounded by history limits
- Minimal CPU overhead when no visualizer is connected
- Thread-safe - can be called from any thread

## Best Practices

1. **Use descriptive names**: `"control/position_error"` vs `"error"`
2. **Organize with categories**: Use `/` to create logical groupings
3. **Include units when relevant**: Helps with visualization and debugging
4. **Log events for major state changes**: Helps with debugging
5. **Use timing functions**: Identify performance bottlenecks
6. **Don't log in hot inner loops**: Unless necessary for debugging

## Troubleshooting

**Q: Telemetry data not showing up?**
- Check that `TelemetryInit::initialize()` was called
- Verify UDP port is not blocked by firewall
- Ensure Python visualizer is running and connected to correct port

**Q: High CPU usage?**
- Reduce publish rate: `TelemetryInit::initialize(port, lowerHz)`
- Avoid logging in tight loops
- Use auto timers instead of manual timing

**Q: Too much data in visualizer?**
- Use categories to organize data
- Focus on specific entries using the dropdown filters
- Clear data periodically if needed

## Examples

See the `examples/` directory for complete working examples:
- `simple_telemetry_example.cpp` - Basic usage
- `telemetry_example.cpp` - Advanced features and integration patterns
