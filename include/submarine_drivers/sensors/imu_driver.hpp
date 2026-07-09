// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/noise_model.hpp"
#include "submarine_drivers/core/sensor_base.hpp"
#include "submarine_drivers/core/types.hpp"
#include <memory>

namespace submarine_drivers {

/**
 * @brief IMU (Inertial Measurement Unit) driver.
 *
 * Simulates a 6-axis IMU providing accelerometer and gyroscope readings
 * in body frame. Supports configurable noise injection for each axis,
 * calibration offsets, and scale factor corrections.
 *
 * Noise model: Gaussian white noise + bias drift per axis, matching
 * real IMU characteristics (e.g., VectorNav VN-100, Xsens MTi).
 */
class ImuDriver : public SensorBase {
public:
    struct Config {
        double update_rate_hz{200.0};        // typical IMU rate

        // Accelerometer noise
        NoiseModel::Config accel_noise;
        Vector3 accel_bias_offset{0.0, 0.0, 0.0};   // calibration offset
        Vector3 accel_scale{1.0, 1.0, 1.0};          // scale factor correction

        // Gyroscope noise
        NoiseModel::Config gyro_noise;
        Vector3 gyro_bias_offset{0.0, 0.0, 0.0};
        Vector3 gyro_scale{1.0, 1.0, 1.0};

        // Gravity vector (NED frame, pointing down)
        double gravity{9.80665};
    };

    ImuDriver() : ImuDriver(Config{}) {}
    explicit ImuDriver(const Config& config);
    ~ImuDriver() override = default;

    // DriverBase interface
    bool initialize() override;
    void update(double dt_seconds) override;
    void shutdown() override;

    // SensorBase interface
    std::chrono::steady_clock::time_point get_last_timestamp() const override;
    bool has_valid_reading() const override;

    /// Get the latest IMU reading
    ImuReading get_reading() const { return last_reading_; }

    /// Set the ground-truth acceleration and angular velocity (for simulation)
    void set_ground_truth(const Vector3& acceleration,
                          const Vector3& angular_velocity);

    /// Get current configuration
    const Config& get_config() const { return config_; }

private:
    Config config_;
    ImuReading last_reading_;
    Vector3 true_acceleration_;
    Vector3 true_angular_velocity_;

    std::unique_ptr<NoiseModel> accel_noise_x_;
    std::unique_ptr<NoiseModel> accel_noise_y_;
    std::unique_ptr<NoiseModel> accel_noise_z_;
    std::unique_ptr<NoiseModel> gyro_noise_x_;
    std::unique_ptr<NoiseModel> gyro_noise_y_;
    std::unique_ptr<NoiseModel> gyro_noise_z_;
};

}  // namespace submarine_drivers
