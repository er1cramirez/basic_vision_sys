#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "ControlTypes.h"
#include "aruco_ekf_estimator.h"
#include <Eigen/Dense>
#include "Logger.h"
#include <chrono>
#include <cmath>
#include <utility>

/**
 * @struct DesiredVelocityResult
 * @brief Result structure for desired velocity calculations
 */
struct DesiredVelocityResult {
    Eigen::Vector3d v_desired;     // Desired velocity vector
    Eigen::Vector3d v_desired_dot; // Derivative of desired velocity
};

/**
 * @struct VelocityPlanningConfig
 * @brief Configuration parameters for desired velocity computation
 */
struct VelocityPlanningConfig {
    double cr;     // Radial velocity gain
    double kt;     // Tangential velocity gain
    double kz;     // Height scaling factor
    double s_min;  // Minimum slope of velocity regulator
    double s_max;  // Maximum slope of velocity regulator
    
    // Default constructor with default values
    VelocityPlanningConfig() 
        : cr(0.2), kt(0.4), kz(2.0), s_min(2.5), s_max(4.0) {}
    
    // Constructor with custom values
    VelocityPlanningConfig(double cr_, double kt_, double kz_, double s_min_, double s_max_)
        : cr(cr_), kt(kt_), kz(kz_), s_min(s_min_), s_max(s_max_) {}
};

/**
 * @brief Calculate desired velocity and its derivative using velocity planning equations
 * @param state Current state estimate
 * @param target Target position (unused for now, but kept for future extensions)
 * @param config Velocity planning configuration parameters
 * @return DesiredVelocityResult containing v_desired and v_desired_dot
 */
inline DesiredVelocityResult computeDesiredVelocity(const EKFStateResult& state, 
                                                   const Eigen::Vector3d& target,
                                                   const VelocityPlanningConfig& config) {

    DesiredVelocityResult result;
    
    // Simplified version of planning equations(No height dependence)
    // Positive height
    double h = state.position.z() - 0.0; //Z offset
    // Height velocity gain: 0 when near to the ground
    // ct = c1 * tanh(c2 * h)
    double ct = config.kt * std::tanh(config.kz * h);
    // Derivative of ct with respect to time
    // ct_dot = c1 * c2 * sech²(c2 * h) * h_dot
    double ct_dot = config.kt * config.kz * (1.0 - std::tanh(config.kz * h) * std::tanh(config.kz * h)) * state.velocity.z();
    // Height dependence for slope factor
    // s = s_min + s_max * exp(-kz * h)
    double s = config.s_min + config.s_max * std::exp(-config.kz * h);
    // Derivative of s with respect to time
    // s_dot = -s_max * kz * exp(-kz * h) * state.velocity.z();
    double s_dot = -config.s_max * config.kz * std::exp(-config.kz * h) * state.velocity.z();
    // Vectorial distance to target
    Eigen::Vector3d d_vector = state.position;
    // discard the z component
    d_vector.z() = 0;
    Eigen::Vector3d d_vector_dot = state.velocity;
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
    // d/dt(mu_r) = d/dt(tanh(s*d)) = sech²(s*d) * (s_dot*d + s*d_dot)
    double mu_r_dot = (1 - mu_r * mu_r) * (s_dot * d + s * d_dot);  // since sech²(s*d) = 1 - tanh²(s*d)

    // d/dt(mu_t) = d/dt(sech(s*d)) = -sech(s*d) * tanh(s*d) * (s_dot*d + s*d_dot)
    double mu_t_dot = -mu_t * mu_r * (s_dot * d + s * d_dot);

    // Desired velocity vector
    result.v_desired = mu_r * config.cr * _R + mu_t * ct * _T;

    // Calculate v_desired_dot
    // d/dt(v_desired) = d/dt(mu_r * cr * _R + mu_t * ct * _T)
    //                 = mu_r_dot * cr * _R + mu_r * cr * R_dot + mu_t_dot * ct * _T + mu_t * ct_dot * _T
    // Note: cr is constant, but ct is height-dependent, so we need ct_dot term
    result.v_desired_dot = mu_r_dot * config.cr * _R + 
                          mu_r * config.cr * R_dot + 
                          mu_t_dot * ct * _T + 
                          mu_t * ct_dot * _T;
    // Note: _T is constant, so its derivative is zero
    UAV::logger().Write("CVDS",
                        "TimeUS,VdX,VdY,VdZ",
                        "Qfff",
                        UAV::logger().getMicroseconds(),
                        result.v_desired.x(),
                        result.v_desired.y(),
                        result.v_desired.z());
    
    return result;
}

/**
 * @class ControllerBase
 * @brief Base class for all controllers
 */
class ControllerBase {
protected:
    // Velocity planning configuration
    VelocityPlanningConfig velocity_config;
    
public:
    ControllerBase() : velocity_config() {
        // Note: Cannot log here because logger might not be initialized yet
        // Logging will be done in derived class initialize() method
    }
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
    
    /**
     * @brief Set velocity planning configuration
     * @param config New configuration parameters
     */
    void setVelocityPlanningConfig(const VelocityPlanningConfig& config) {
        velocity_config = config;
        // Log the new configuration
        logVelocityPlanningConfig();
    }
    
    /**
     * @brief Get current velocity planning configuration
     * @return Current configuration parameters
     */
    VelocityPlanningConfig getVelocityPlanningConfig() const {
        return velocity_config;
    }

protected:
    /**
     * @brief Log velocity planning configuration
     */
    void logVelocityPlanningConfig() {
        // Only log if logger is initialized
        if (UAV::logger().isInitialized()) {
            UAV::logger().Write("CVGA",
                                "TimeUS,Cr,Kt,Kz,Smin,Smax",
                                "Qfffff",
                                UAV::logger().getMicroseconds(),
                                velocity_config.cr,
                                velocity_config.kt,
                                velocity_config.kz,
                                velocity_config.s_min,
                                velocity_config.s_max);
        }
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
    
    // Timing
    std::chrono::steady_clock::time_point last_update_time;
    
    // Controller gains (per axis)
    Eigen::Vector3d kp;
    Eigen::Vector3d ki;
    
public:
    VelocityPIController(const Eigen::Vector3d& proportional_gain = Eigen::Vector3d(-0.01, -0.01, -0.08), 
                        const Eigen::Vector3d& integral_gain = Eigen::Vector3d(0.0, 0.0, 0.0)) 
        : kp(proportional_gain), ki(integral_gain), has_previous_error(false) {
        integral_error = Eigen::Vector3d::Zero();
        previous_error = Eigen::Vector3d::Zero();
        last_update_time = std::chrono::steady_clock::now();
        // Note: Cannot log here because logger might not be initialized yet
        // Logging moved to initialize() method
    }
    
    ~VelocityPIController() override = default;
    
    bool initialize() override {
        reset();
        
        // Log velocity planning configuration from base class
        logVelocityPlanningConfig();
        
        // Log controller gains
        UAV::logger().Write("VCGA", "TimeUS,Kpx,Kpy,Kpz,Kix,Kiy,Kiz",
                           "Qffffff", UAV::logger().getMicroseconds(),
                           kp.x(), kp.y(), kp.z(),
                           ki.x(), ki.y(), ki.z());
        
        return true;
    }
    
    void reset() override {
        // Reset controller state
        integral_error = Eigen::Vector3d::Zero();
        previous_error = Eigen::Vector3d::Zero();
        has_previous_error = false;
        last_update_time = std::chrono::steady_clock::now();
    }
    
    /**
     * @brief Set controller gains for all axes
     * @param proportional_gain Proportional gains [kp_x, kp_y, kp_z]
     * @param integral_gain Integral gains [ki_x, ki_y, ki_z]
     */
    void setGains(const Eigen::Vector3d& proportional_gain, const Eigen::Vector3d& integral_gain) {
        kp = proportional_gain;
        ki = integral_gain;
    }
    
    /**
     * @brief Set controller gains for individual axis
     * @param axis Axis index (0=x, 1=y, 2=z)
     * @param proportional_gain Proportional gain for specified axis
     * @param integral_gain Integral gain for specified axis
     */
    void setAxisGains(int axis, double proportional_gain, double integral_gain) {
        if (axis >= 0 && axis < 3) {
            kp[axis] = proportional_gain;
            ki[axis] = integral_gain;
        }
    }
    
    /**
     * @brief Get current controller gains
     * @return Pair of (kp, ki) vectors
     */
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getGains() const {
        return std::make_pair(kp, ki);
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

        // Calculate desired velocity and its derivative using base config
        DesiredVelocityResult desired = computeDesiredVelocity(state, target, velocity_config);
        
        // Calculate velocity error and its derivative PER AXIS for debugging
        Eigen::Vector3d v_drone = -state.velocity;
        Eigen::Vector3d a_drone = -state.acceleration;

        Eigen::Vector3d error = v_drone - desired.v_desired;
        Eigen::Vector3d error_dot = a_drone - desired.v_desired_dot;
        // Debug: Separate calculations by axis
        double error_x = v_drone.x() - desired.v_desired.x();
        double error_y = v_drone.y() - desired.v_desired.y();
        double error_z = v_drone.z() - desired.v_desired.z();
        
        double error_dot_x = a_drone.x() - desired.v_desired_dot.x();
        double error_dot_y = a_drone.y() - desired.v_desired_dot.y();
        double error_dot_z = a_drone.z() - desired.v_desired_dot.z();

        // Update integral term only if dt is reasonable
        if (dt > 0.0 && dt < 0.1) { // Avoid integration if dt is too large (likely first call or long pause)
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
                
                // Apply integration to all axes
                integral_error += delta_integral;
            } else {
                // First iteration: use simple Euler since we don't have previous error
                integral_error += error * dt;
                has_previous_error = true;
            }
            
            // Store current error for next iteration
            previous_error = error;
            
            // Anti-windup: limit integral error to prevent excessive accumulation - PER AXIS
            double max_integral = 3; // Maximum integral error magnitude
            
            // X-axis anti-windup
            if (integral_error[0] > max_integral) {
                integral_error[0] = max_integral;
            } else if (integral_error[0] < -max_integral) {
                integral_error[0] = -max_integral;
            }
            
            // Y-axis anti-windup
            if (integral_error[1] > max_integral) {
                integral_error[1] = max_integral;
            } else if (integral_error[1] < -max_integral) {
                integral_error[1] = -max_integral;
            }
            
            // Z-axis anti-windup
            if (integral_error[2] > max_integral) {
                integral_error[2] = max_integral;
            } else if (integral_error[2] < -max_integral) {
                integral_error[2] = -max_integral;
            }
        }

        // PI Control - SEPARATED BY AXIS for debugging
        // X-axis control
        double proportional_term_x = error_x * kp.x();
        double integral_term_x = integral_error.x() * ki.x();
        double u_desired_x = proportional_term_x + integral_term_x;
        double u_desired_dot_x = error_dot_x * kp.x() + error_x * ki.x();
        
        // Y-axis control
        double proportional_term_y = error_y * kp.y();
        double integral_term_y = integral_error.y() * ki.y();
        double u_desired_y = proportional_term_y + integral_term_y;
        double u_desired_dot_y = error_dot_y * kp.y() + error_y * ki.y();
        
        // Z-axis control
        double proportional_term_z = error_z * kp.z();
        double integral_term_z = integral_error.z() * ki.z();
        double u_desired_z = proportional_term_z + integral_term_z;
        double u_desired_dot_z = error_dot_z * kp.z() + error_z * ki.z();
        
        // Special handling for landing when height < 0.3m
        if (state.position.z() < 0.3) {
            // Set a small constant positive value to maintain gentle descent
            u_desired_z = 0.1;  // This value can be tuned based on your needs
            u_desired_dot_z = 0.0;  // No acceleration during final landing phase
        }
        
        // Debug control outputs by axis
        // u_desired_x = 0.0;
        // u_desired_y = 0.0015;
        // u_desired_z = 0.01;
        // Assemble final output
        output.u_desired = Eigen::Vector3d(u_desired_x, u_desired_y, u_desired_z);
        output.u_desired_dot = Eigen::Vector3d(u_desired_dot_x, u_desired_dot_y, u_desired_dot_z);

        // Update timestamp
        last_update_time = current_time;
        // Control performance by axis
        UAV::logger().Write("VCPX", "TimeUS,Vd_x,V_x,Verr_x",
                           "Qfff", UAV::logger().getMicroseconds(),
                           desired.v_desired.x(), -state.velocity.x(), error_x);
        UAV::logger().Write("VCPY", "TimeUS,Vd_y,V_y,Verr_y",
                           "Qfff", UAV::logger().getMicroseconds(),
                           desired.v_desired.y(), -state.velocity.y(), error_y);
        UAV::logger().Write("VCPZ", "TimeUS,Vd_z,V_z,Verr_z",
                           "Qfff", UAV::logger().getMicroseconds(),
                           desired.v_desired.z(), -state.velocity.z(), error_z);

        // Log control commands by axis
        UAV::logger().Write("CPUX", "TimeUS,Ux,Px,Ix",
                           "Qfff", UAV::logger().getMicroseconds(),
                           output.u_desired.x(), proportional_term_x, integral_term_x);
        UAV::logger().Write("CPUY", "TimeUS,Uy,Py,Iy",
                           "Qfff", UAV::logger().getMicroseconds(),
                           output.u_desired.y(), proportional_term_y, integral_term_y);
        UAV::logger().Write("CPUZ", "TimeUS,Uz,Pz,Iz",
                           "Qfff", UAV::logger().getMicroseconds(),
                           output.u_desired.z(), proportional_term_z, integral_term_z);

        // Enhanced logging with per-axis breakdown for debugging
        UAV::logger().Write("CPRM", "TimeUS,Verrx,Verry,Verrz,Ierrx,Ierry,Ierrz,Ux,Uy,Uz,Px,Py,Pz,Ix,Iy,Iz",
                           "Qfffffffffffffff", output.timestamp,
                           error_x, error_y, error_z,
                           integral_error.x(), integral_error.y(), integral_error.z(),
                           u_desired_x, u_desired_y, u_desired_z,
                           proportional_term_x, proportional_term_y, proportional_term_z,
                           integral_term_x, integral_term_y, integral_term_z);

        return output;
    }
};

#endif // CONTROLLERS_H