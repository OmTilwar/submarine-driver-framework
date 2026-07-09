// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/types.hpp"
#include "submarine_drivers/sensors/imu_driver.hpp"
#include "submarine_drivers/sensors/dvl_driver.hpp"
#include "submarine_drivers/sensors/depth_sensor_driver.hpp"
#include <array>
#include <memory>

namespace submarine_drivers {

/**
 * @brief Simplified EKF-based 6-DOF state estimator for underwater vehicles.
 *
 * Fuses IMU (high rate), DVL (medium rate), and depth sensor (medium rate)
 * data into a unified 6-DOF vehicle pose estimate:
 *   State = [x, y, z, roll, pitch, yaw, u, v, w, p, q, r]
 *   (position, orientation, linear velocity, angular velocity)
 *
 * Uses a simplified Extended Kalman Filter:
 *   - Prediction: IMU-driven dead reckoning (accelerometer + gyroscope)
 *   - Update: DVL velocity correction + depth sensor z-correction
 *
 * This is a teaching/demonstration implementation. Production EKFs would
 * use Eigen matrices for the full state covariance propagation.
 *
 * Note: For a production submarine, this would also incorporate GPS
 * (when surfaced), USBL, and compass data.
 */
class StateEstimator6DOF {
public:
    /// 12-element state vector: [x,y,z, roll,pitch,yaw, u,v,w, p,q,r]
    static constexpr int STATE_DIM = 12;
    using StateVector = std::array<double, STATE_DIM>;

    /// State indices
    enum StateIndex : int {
        X = 0, Y = 1, Z = 2,           // position (NED frame)
        ROLL = 3, PITCH = 4, YAW = 5,  // orientation (Euler angles, rad)
        U = 6, V = 7, W = 8,           // linear velocity (body frame)
        P = 9, Q = 10, R = 11          // angular velocity (body frame)
    };

    struct Config {
        // Process noise (prediction uncertainty per second)
        double position_process_noise{0.01};
        double orientation_process_noise{0.005};
        double velocity_process_noise{0.1};
        double angular_rate_process_noise{0.05};

        // Measurement noise (observation uncertainty)
        double dvl_velocity_noise{0.02};    // m/s
        double depth_noise{0.05};           // meters
        double imu_accel_noise{0.1};        // m/s^2
        double imu_gyro_noise{0.01};        // rad/s

        // Gravity
        double gravity{9.80665};
    };

    explicit StateEstimator6DOF(const Config& config = Config{});

    /// Reset state to zero (or specified initial state)
    void reset(const StateVector& initial_state = StateVector{});

    /// Prediction step: propagate state using IMU data
    void predict(const ImuReading& imu, double dt_seconds);

    /// Update step: correct state using DVL velocity measurement
    void update_dvl(const DvlReading& dvl);

    /// Update step: correct state using depth measurement
    void update_depth(const DepthReading& depth);

    /// Get current state estimate
    const StateVector& get_state() const { return state_; }

    /// Get current pose as Pose6DOF
    Pose6DOF get_pose() const;

    /// Get current velocity
    Velocity6DOF get_velocity() const;

    /// Get diagonal of covariance (uncertainty per state element)
    const StateVector& get_covariance_diagonal() const { return covariance_diag_; }

    /// Get the innovation (measurement - predicted measurement) from last update
    double get_last_innovation() const { return last_innovation_norm_; }

private:
    Config config_;
    StateVector state_;
    StateVector covariance_diag_;  // simplified: diagonal covariance only
    double last_innovation_norm_{0.0};

    /// Rotate body-frame vector to NED frame using current orientation
    Vector3 body_to_ned(const Vector3& body_vec) const;

    /// Normalize angle to [-pi, pi]
    static double normalize_angle(double angle);

    /// Simplified Kalman gain for scalar measurement
    double kalman_gain(int state_idx, double measurement_noise) const;

    /// Apply scalar measurement update to state and covariance
    void scalar_update(int state_idx, double innovation,
                       double measurement_noise);
};

}  // namespace submarine_drivers
