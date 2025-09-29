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
                                                   double cr = 0.8, 
                                                   double kt = 0.5, 
                                                   double kz = 2.0,
                                                   double s_min = 2.5,
                                                   double s_max = 4.0) {
    DesiredVelocityResult result;
    
    // Simplified version of planning equations(No height dependence)
    // Positive height
    double h = state.position.z() - 0.75; //Z offset
    // Height velocity gain: 0 when near to the ground
    // ct = c1 * tanh(c2 * h)
    double ct = kt * std::tanh(kz * h);
    // Derivative of ct with respect to time
    // ct_dot = c1 * c2 * sech²(c2 * h) * h_dot
    double ct_dot = kt * kz * (1.0 - std::tanh(kz * h) * std::tanh(kz * h)) * state.velocity.z();
    // Height dependence for slope factor
    // s = s_min + s_max * exp(-kz * h)
    double s = s_min + s_max * std::exp(-kz * h);
    // Derivative of s with respect to time
    // s_dot = -s_max * kz * exp(-kz * h) * state.velocity.z();
    double s_dot = -s_max * kz * std::exp(-kz * h) * state.velocity.z();
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
    Eigen::Vector3d _T = Eigen::Vector3d(0, 0, -1);

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
    result.v_desired = mu_r * cr * _R + mu_t * ct * _T;

    // Calculate v_desired_dot
    // d/dt(v_desired) = d/dt(mu_r * cr * _R + mu_t * ct * _T)
    //                 = mu_r_dot * cr * _R + mu_r * cr * R_dot + mu_t_dot * ct * _T + mu_t * ct_dot * _T
    // Note: cr is constant, but ct is height-dependent, so we need ct_dot term
    result.v_desired_dot = mu_r_dot * cr * _R + 
                          mu_r * cr * R_dot + 
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
    double delay_time; // Delay in seconds before integral term starts accumulating (for X and Y axes)
    Eigen::Vector3i integral_active; // Per-axis integral activation (0=inactive, 1=active)
    
    // Controller gains (per axis)
    Eigen::Vector3d kp;
    Eigen::Vector3d ki;
    
public:
    VelocityPIController(const Eigen::Vector3d& proportional_gain = Eigen::Vector3d(-0.01, -0.01, 0.02), 
                        const Eigen::Vector3d& integral_gain = Eigen::Vector3d(-0.001, -0.001, 0.0), 
                        double integral_delay = 3.0) 
        : kp(proportional_gain), ki(integral_gain), delay_time(integral_delay), has_previous_error(false) {
        integral_error = Eigen::Vector3d::Zero();
        previous_error = Eigen::Vector3d::Zero();
        // Z-axis integral is active from start, X and Y axes start inactive
        integral_active = Eigen::Vector3i(0, 0, 1); // [x_inactive, y_inactive, z_active]
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
        // Z-axis integral is active from start, X and Y axes start inactive
        integral_active = Eigen::Vector3i(0, 0, 1); // [x_inactive, y_inactive, z_active]
        start_time = std::chrono::steady_clock::now();
        last_update_time = start_time;
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
     * @brief Set integral delay time
     * @param delay Delay in seconds for X and Y axes (Z-axis is always active)
     */
    void setIntegralDelay(double delay) {
        delay_time = delay;
    }
    
    /**
     * @brief Get integral activation status per axis
     * @return Vector indicating which axes have active integral terms [x, y, z]
     */
    Eigen::Vector3i getIntegralStatus() const {
        return integral_active;
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
        
        // Check if enough time has passed to activate integral term for X and Y axes
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
        if (elapsed_time >= delay_time) {
            integral_active[0] = 1; // Activate X-axis integral
            integral_active[1] = 1; // Activate Y-axis integral
            // Z-axis is already active from start (integral_active[2] = 1)
        }

        // Calculate desired velocity and its derivative
        DesiredVelocityResult desired = computeDesiredVelocity(state, target);
        
        // Calculate velocity error and its derivative PER AXIS for debugging
        Eigen::Vector3d error = state.velocity - desired.v_desired;
        Eigen::Vector3d error_dot = state.acceleration - desired.v_desired_dot;
        
        // Debug: Separate calculations by axis
        double error_x = error.x();
        double error_y = error.y(); 
        double error_z = error.z();
        
        double error_dot_x = error_dot.x();
        double error_dot_y = error_dot.y();
        double error_dot_z = error_dot.z();

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
                
                // Apply integration only to active axes - SEPARATED BY AXIS
                if (integral_active[0]) { // X-axis
                    integral_error[0] += delta_integral[0];
                }
                if (integral_active[1]) { // Y-axis
                    integral_error[1] += delta_integral[1];
                }
                if (integral_active[2]) { // Z-axis
                    integral_error[2] += delta_integral[2];
                }
            } else {
                // First iteration: use simple Euler since we don't have previous error
                // Apply integration only to active axes - SEPARATED BY AXIS
                if (integral_active[0]) { // X-axis
                    integral_error[0] += error_x * dt;
                }
                if (integral_active[1]) { // Y-axis
                    integral_error[1] += error_y * dt;
                }
                if (integral_active[2]) { // Z-axis
                    integral_error[2] += error_z * dt;
                }
                has_previous_error = true;
            }
            
            // Store current error for next iteration
            previous_error = error;
            
            // Anti-windup: limit integral error to prevent excessive accumulation - PER AXIS
            double max_integral = 0.1; // Maximum integral error magnitude
            
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
        
        // Assemble final output
        output.u_desired = Eigen::Vector3d(u_desired_x, u_desired_y, u_desired_z);
        output.u_desired_dot = Eigen::Vector3d(u_desired_dot_x, u_desired_dot_y, u_desired_dot_z);

        // Update timestamp
        last_update_time = current_time;
        // Control performance by axis
        UAV::logger().Write("CPRZ", "TimeUS,Vd_z,V_z,Verr_z,Uz",
                           "Qffff", UAV::logger().getMicroseconds(),
                           desired.v_desired.z(), state.velocity.z(), error_z, output.u_desired.z());

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