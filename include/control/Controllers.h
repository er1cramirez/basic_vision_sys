#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "ControlTypes.h"
#include "aruco_ekf_estimator.h"
#include <Eigen/Dense>
#include "Logger.h"
#include <chrono>
#include <cmath>

/**
 * @struct DesiredVelocityResult
 * @brief Result structure for desired velocity calculations
 */
struct DesiredVelocityResult {
    Eigen::Vector3d v_desired;     // Desired velocity vector
    Eigen::Vector3d v_desired_dot; // Derivative of desired velocity
};

/**
 * @brief Calculate desired velocity and its derivative using velocity planning equations
 * @param state Current state estimate
 * @param target Target position (unused for now, but kept for future extensions)
 * @param cr Radial velocity gain
 * @param ct Tangential velocity gain  
 * @param s Slope of the velocity regulator
 * @return DesiredVelocityResult containing v_desired and v_desired_dot
 */
inline DesiredVelocityResult computeDesiredVelocity(const EKFStateResult& state, 
                                                   const Eigen::Vector3d& target,
                                                   double cr = 0.2, 
                                                   double kt = 0.0, 
                                                   double kz = 0.1,
                                                   double s_min = 1.5,
                                                   double s_max = 2.0) {
    DesiredVelocityResult result;
    
    // Simplified version of planning equations(No height dependence)
    // Positive height
    double h = state.position.z();
    // Height velocity gain: 0 when near to the ground
    // ct = c1 * tanh(c2 * h)
    double ct = kt * std::tanh(kz * h);
    // Height dependence for slope factor
    // s = s_min + s_max * exp(-kz * h)
    double s = s_min + s_max * std::exp(-kz * h);
    // Vectorial distance to target
    Eigen::Vector3d d_vector = -state.position;
    // discard the z component
    d_vector.z() = 0;
    Eigen::Vector3d d_vector_dot = -state.velocity;
    // discard the z component
    d_vector_dot.z() = 0;

    // Unit vector to target
    Eigen::Vector3d _R = d_vector.normalized();
    // Scalar distance to target
    double d = d_vector.norm();

    // Derivative of d respect to time
    // d/dt d = R' * d_vector_dot
    // where R' is the transpose of R
    double d_dot = _R.dot(d_vector_dot);

    // Then for R_dot we have:
    // R_dot = (I - R * R') * d_vector_dot / d_vector.norm()
    // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    // Eigen::Matrix3d R_RT = _R * _R.transpose();
    // Eigen::Vector3d R_dot = (I - R_RT) * d_vector_dot / d;

    // Alternative (more efficient) computation:
    // R_dot = (d_vector_dot - _R * (_R.dot(d_vector_dot))) / d
    // Which is equivalent to:
    Eigen::Vector3d R_dot = (d_vector_dot - _R * d_dot) / d;

    // Tangential unity vector (z axis)
    Eigen::Vector3d _T = Eigen::Vector3d(0, 0, 1);

    // Radial velocity regulator mu_r = tanh(d)
    double mu_r = std::tanh(s * d);
    // Tangential velocity regulator mu_t = sech(d)
    double mu_t = 1 / std::cosh(s * d);

    // Calculate derivatives of the velocity regulators
    // d/dt(mu_r) = d/dt(tanh(d)) = sech²(d) * d_dot
    double mu_r_dot = (1 - mu_r * mu_r) * d_dot;  // since sech²(d) = 1 - tanh²(d)

    // d/dt(mu_t) = d/dt(sech(d)) = -sech(d) * tanh(d) * d_dot
    double mu_t_dot = -mu_t * mu_r * d_dot;

    // Desired velocity vector
    result.v_desired = mu_r * cr * _R + mu_t * ct * _T;

    // Calculate v_desired_dot
    // d/dt(v_desired) = d/dt(mu_r * cr * _R + mu_t * ct * _T)
    //                 = mu_r_dot * cr * _R + mu_r * cr * R_dot + mu_t_dot * ct * _T
    result.v_desired_dot = mu_r_dot * cr * _R + 
                          mu_r * cr * R_dot + 
                          mu_t_dot * ct * _T;
    // Note: _T is constant, so its derivative is zero
    
    return result;
}

/**
 * @class ControllerBase
 * @brief Base class for all controllers
 */
class ControllerBase {
public:
    ControllerBase() = default;
    virtual ~ControllerBase() = default;
    
    /**
     * @brief Initialize the controller
     * @return True if initialization was successful
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief Reset the controller
     */
    virtual void reset() = 0;
};

/**
 * @class PositionController
 * @brief Controller for position control
 */
class PositionController : public ControllerBase {
public:
    PositionController() = default;
    ~PositionController() override = default;
    
    bool initialize() override {
        return true;
    }
    
    void reset() override {
        // Reset controller state
    }
    
    /**
     * @brief Compute control output from state estimate
     * @param state Current state estimate
     * @param target Target position
     * @return Control output
     */
    ControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& target) {
        ControlOutput output;
        
        // Simple P controller for position
        Eigen::Vector3d error = target - state.position;
        double kp = 0.02;
        double kd = 0.035;
        output.u_desired = error * kp + (-state.velocity * kd);
        
        output.u_desired_dot = Eigen::Vector3d::Zero(); // No acceleration control
        
        return output;
    }
};

/**
 * @class VelocityController
 * @brief Controller for velocity control
 */
class VelocityController : public ControllerBase {
public:
    VelocityController() = default;
    ~VelocityController() override = default;
    
    bool initialize() override {
        return true;
    }
    
    void reset() override {
        // Reset controller state
    }
    
    /**
     * @brief Compute control output from state estimate
     * @param state Current state estimate
     * @param target Target position
     * @return Control output
     */
    ControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& target) {
        ControlOutput output;

        // Calculate desired velocity and its derivative
        DesiredVelocityResult desired = computeDesiredVelocity(state, target);
        
        // Calculate velocity error and its derivative
        Eigen::Vector3d error = state.velocity - desired.v_desired;
        Eigen::Vector3d error_dot = state.acceleration - desired.v_desired_dot;  // assuming you have acceleration in state

        // Control and its derivative
        double kp = -0.01;
        output.u_desired = error * kp;
        output.u_desired_dot = error_dot * kp;

        UAV::logger().Write("CPRM", "TimeUS,Verrx,Verry,Verrz,Ux,Uy,Uz",
                           "Qffffff", output.timestamp,
                           error.x(), error.y(), error.z(),
                           output.u_desired.x(), output.u_desired.y(), output.u_desired.z());

        return output;
    }
};

/**
 * @class VelocityPIController
 * @brief PI controller for velocity control with integral term and delay handling
 */
class VelocityPIController : public ControllerBase {
private:
    // Integral term storage
    Eigen::Vector3d integral_error;
    
    // Previous error for RK4 integration
    Eigen::Vector3d previous_error;
    bool has_previous_error;
    
    // Delay mechanism
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point last_update_time;
    double delay_time; // Delay in seconds before integral term starts accumulating
    bool integral_active;
    
    // Controller gains
    double kp;
    double ki;
    
public:
    VelocityPIController(double proportional_gain = -0.01, double integral_gain = -0.005, double integral_delay = 2.0) 
        : kp(proportional_gain), ki(integral_gain), delay_time(integral_delay), integral_active(false), has_previous_error(false) {
        integral_error = Eigen::Vector3d::Zero();
        previous_error = Eigen::Vector3d::Zero();
        start_time = std::chrono::steady_clock::now();
        last_update_time = start_time;
    }
    
    ~VelocityPIController() override = default;
    
    bool initialize() override {
        reset();
        return true;
    }
    
    void reset() override {
        // Reset controller state
        integral_error = Eigen::Vector3d::Zero();
        previous_error = Eigen::Vector3d::Zero();
        has_previous_error = false;
        integral_active = false;
        start_time = std::chrono::steady_clock::now();
        last_update_time = start_time;
    }
    
    /**
     * @brief Set controller gains
     * @param proportional_gain Proportional gain
     * @param integral_gain Integral gain
     */
    void setGains(double proportional_gain, double integral_gain) {
        kp = proportional_gain;
        ki = integral_gain;
    }
    
    /**
     * @brief Set integral delay time
     * @param delay Delay in seconds
     */
    void setIntegralDelay(double delay) {
        delay_time = delay;
    }
    
    /**
     * @brief Compute control output from state estimate with PI control
     * @param state Current state estimate
     * @param target Target position
     * @return Control output
     */
    ControlOutput computeControl(const EKFStateResult& state, const Eigen::Vector3d& target) {
        ControlOutput output;
        
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_update_time).count();
        
        // Check if enough time has passed to activate integral term
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
        if (!integral_active && elapsed_time >= delay_time) {
            integral_active = true;
        }

        // Calculate desired velocity and its derivative
        DesiredVelocityResult desired = computeDesiredVelocity(state, target);
        
        // Calculate velocity error and its derivative
        Eigen::Vector3d error = state.velocity - desired.v_desired;
        Eigen::Vector3d error_dot = state.acceleration - desired.v_desired_dot;  // assuming you have acceleration in state

        // Update integral term only if delay has passed and dt is reasonable
        if (integral_active && dt > 0.0 && dt < 0.1) { // Avoid integration if dt is too large (likely first call or long pause)
            // Fourth-order Runge-Kutta integration for the integral term
            // We're integrating: d(integral_error)/dt = error
            // The derivative function is f(t, integral_error) = error
            
            if (has_previous_error) {
                // More accurate RK4 with error trend estimation
                // Estimate error derivative for better k2, k3, k4 approximations (numerical derivative)
                Eigen::Vector3d error_trend = (error - previous_error) / dt;
                
                // k1 = f(t, y) = error at current time
                Eigen::Vector3d k1 = error;
                
                // k2 = f(t + dt/2, y + k1*dt/2)
                // Approximate error at t + dt/2 using linear extrapolation
                Eigen::Vector3d error_half = error + error_trend * (dt / 2.0);
                Eigen::Vector3d k2 = error_half;
                
                // k3 = f(t + dt/2, y + k2*dt/2)
                // Use the same error_half approximation
                Eigen::Vector3d k3 = error_half;
                
                // k4 = f(t + dt, y + k3*dt)
                // Approximate error at t + dt using linear extrapolation
                Eigen::Vector3d error_full = error + error_trend * dt;
                Eigen::Vector3d k4 = error_full;
                
                // RK4 formula: y_{n+1} = y_n + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
                Eigen::Vector3d delta_integral = (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
                integral_error += delta_integral;
            } else {
                // First iteration: use simple Euler since we don't have previous error
                integral_error += error * dt;
                has_previous_error = true;
            }
            
            // Store current error for next iteration
            previous_error = error;
            
            // Anti-windup: limit integral error to prevent excessive accumulation
            double max_integral = 0.1; // Maximum integral error magnitude
            for (int i = 0; i < 3; ++i) {
                if (integral_error[i] > max_integral) {
                    integral_error[i] = max_integral;
                } else if (integral_error[i] < -max_integral) {
                    integral_error[i] = -max_integral;
                }
            }
        }

        // PI Control
        Eigen::Vector3d proportional_term = error * kp;
        Eigen::Vector3d integral_term = integral_error * ki;
        
        output.u_desired = proportional_term + integral_term;
        // Correct derivative: u_dot = kp * error_dot + ki * error
        // Since d/dt(integral_error) = error
        output.u_desired_dot = error_dot * kp + error * ki;

        // Update timestamp
        last_update_time = current_time;

        UAV::logger().Write("CPRM", "TimeUS,Verrx,Verry,Verrz,Ierrx,Ierry,Ierrz,Ux,Uy,Uz",
                           "Qfffffffff", output.timestamp,
                           error.x(), error.y(), error.z(),
                           integral_error.x(), integral_error.y(), integral_error.z(),
                           output.u_desired.x(), output.u_desired.y(), output.u_desired.z());

        return output;
    }
};

#endif // CONTROLLERS_H