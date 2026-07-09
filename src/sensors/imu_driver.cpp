// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/sensors/imu_driver.hpp"

namespace submarine_drivers {

ImuDriver::ImuDriver(const Config& config)
    : SensorBase("imu_driver", config.update_rate_hz)
    , config_(config) {}

bool ImuDriver::initialize() {
    status_ = DriverStatus::INITIALIZING;

    // Create per-axis noise models for accelerometer
    accel_noise_x_ = std::make_unique<NoiseModel>(config_.accel_noise);
    accel_noise_y_ = std::make_unique<NoiseModel>(config_.accel_noise);
    accel_noise_z_ = std::make_unique<NoiseModel>(config_.accel_noise);

    // Create per-axis noise models for gyroscope
    gyro_noise_x_ = std::make_unique<NoiseModel>(config_.gyro_noise);
    gyro_noise_y_ = std::make_unique<NoiseModel>(config_.gyro_noise);
    gyro_noise_z_ = std::make_unique<NoiseModel>(config_.gyro_noise);

    // Initialize with gravity on z-axis (vehicle at rest, NED frame)
    true_acceleration_ = Vector3(0.0, 0.0, config_.gravity);
    true_angular_velocity_ = Vector3(0.0, 0.0, 0.0);

    last_reading_.valid = false;
    status_ = DriverStatus::RUNNING;
    return true;
}

void ImuDriver::update(double dt_seconds) {
    if (status_ != DriverStatus::RUNNING) return;

    ImuReading reading;
    reading.timestamp = std::chrono::steady_clock::now();

    // Apply calibration: offset + scale factor
    double ax = (true_acceleration_.x + config_.accel_bias_offset.x) *
                config_.accel_scale.x;
    double ay = (true_acceleration_.y + config_.accel_bias_offset.y) *
                config_.accel_scale.y;
    double az = (true_acceleration_.z + config_.accel_bias_offset.z) *
                config_.accel_scale.z;

    // Apply noise to accelerometer
    reading.acceleration.x = accel_noise_x_->apply(ax, dt_seconds);
    reading.acceleration.y = accel_noise_y_->apply(ay, dt_seconds);
    reading.acceleration.z = accel_noise_z_->apply(az, dt_seconds);

    // Apply calibration + noise to gyroscope
    double gx = (true_angular_velocity_.x + config_.gyro_bias_offset.x) *
                config_.gyro_scale.x;
    double gy = (true_angular_velocity_.y + config_.gyro_bias_offset.y) *
                config_.gyro_scale.y;
    double gz = (true_angular_velocity_.z + config_.gyro_bias_offset.z) *
                config_.gyro_scale.z;

    reading.angular_velocity.x = gyro_noise_x_->apply(gx, dt_seconds);
    reading.angular_velocity.y = gyro_noise_y_->apply(gy, dt_seconds);
    reading.angular_velocity.z = gyro_noise_z_->apply(gz, dt_seconds);

    reading.valid = true;
    last_reading_ = reading;
}

void ImuDriver::shutdown() {
    status_ = DriverStatus::SHUTDOWN;
    last_reading_.valid = false;
}

std::chrono::steady_clock::time_point ImuDriver::get_last_timestamp() const {
    return last_reading_.timestamp;
}

bool ImuDriver::has_valid_reading() const {
    return last_reading_.valid;
}

void ImuDriver::set_ground_truth(const Vector3& acceleration,
                                  const Vector3& angular_velocity) {
    true_acceleration_ = acceleration;
    true_angular_velocity_ = angular_velocity;
}

}  // namespace submarine_drivers
