# VelocityPIController

## Overview

The `VelocityPIController` is an enhanced version of the `VelocityController` that includes an integral term to improve steady-state error elimination. It implements a PI (Proportional-Integral) controller with special features to handle simulation delays and prevent integral windup.

## Key Features

### 1. PI Control
- **Proportional Term**: Provides immediate response to velocity errors
- **Integral Term**: Eliminates steady-state errors by accumulating error over time
- **Configurable Gains**: Both proportional (kp) and integral (ki) gains can be adjusted

### 2. Delay Mechanism
- **Integral Delay**: Prevents integral term activation for a specified period after initialization
- **Purpose**: Avoids excessive accumulation during simulation startup when the system might not be stable
- **Default Delay**: 2.0 seconds (configurable)

### 3. Anti-Windup Protection
- **Integral Clamping**: Limits the integral error to prevent excessive accumulation
- **Maximum Integral**: Configurable maximum integral error magnitude (default: 10.0)
- **Time Step Validation**: Only integrates when time step is reasonable (prevents large jumps)

## Usage

### Basic Usage

```cpp
// Create controller with default parameters
VelocityPIController controller;

// Initialize
controller.initialize();

// Use in control loop
ControlOutput output = controller.computeControl(state, target);
```

### Custom Configuration

```cpp
// Create controller with custom gains and delay
double kp = -0.01;  // Proportional gain
double ki = -0.005; // Integral gain  
double delay = 3.0; // 3 seconds delay

VelocityPIController controller(kp, ki, delay);

// Or set gains after creation
controller.setGains(-0.015, -0.008);
controller.setIntegralDelay(2.5);
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp` | -0.01 | Proportional gain |
| `ki` | -0.005 | Integral gain |
| `delay_time` | 2.0 | Delay before integral term activates (seconds) |
| `max_integral` | 10.0 | Maximum integral error magnitude |

## Implementation Details

### Integral Term Calculation
The integral term is calculated using simple rectangular integration (Euler method):
```cpp
integral_error += error * dt;
```

### Delay Logic
```cpp
double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
if (!integral_active && elapsed_time >= delay_time) {
    integral_active = true;
}
```

### Anti-Windup
```cpp
// Limit integral error to prevent excessive accumulation
for (int i = 0; i < 3; ++i) {
    if (integral_error[i] > max_integral) {
        integral_error[i] = max_integral;
    } else if (integral_error[i] < -max_integral) {
        integral_error[i] = -max_integral;
    }
}
```

## Logging

The controller logs the following data:
- Velocity errors (x, y, z)
- Integral errors (x, y, z)  
- Control outputs (x, y, z)
- Timestamp

Log format: `"TimeUS,Verrx,Verry,Verrz,Ierrx,Ierry,Ierrz,Ux,Uy,Uz"`

## Comparison with VelocityController

| Feature | VelocityController | VelocityPIController |
|---------|-------------------|---------------------|
| Control Type | P (Proportional) | PI (Proportional-Integral) |
| Steady-state Error | May have offset | Eliminates steady-state error |
| Startup Behavior | Immediate response | Delayed integral activation |
| Windup Protection | N/A | Anti-windup clamping |
| Complexity | Simple | Moderate |

## When to Use

- **Use VelocityPIController when**:
  - Steady-state accuracy is important
  - You can tolerate slightly more complex behavior
  - The system has consistent delays or disturbances

- **Use VelocityController when**:
  - Simple, fast response is sufficient
  - Steady-state errors are acceptable
  - You want predictable, linear behavior

## Example

See `examples/velocity_pi_controller_example.cpp` for a complete working example.
