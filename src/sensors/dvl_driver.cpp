// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/sensors/dvl_driver.hpp"

namespace submarine_drivers {

DvlDriver::DvlDriver(const Config& config)
    : SensorBase("dvl_driver", config.update_rate_hz)
    , config_(config) {}

bool DvlDriver::initialize() {
    status_ = DriverStatus::INITIALIZING;

    vel_noise_x_ = std::make_unique<NoiseModel>(config_.velocity_noise);
    vel_noise_y_ = std::make_unique<NoiseModel>(config_.velocity_noise);
    vel_noise_z_ = std::make_unique<NoiseModel>(config_.velocity_noise);
    alt_noise_ = std::make_unique<NoiseModel>(config_.altitude_noise);

    true_velocity_ = Vector3(0.0, 0.0, 0.0);
    true_altitude_ = 50.0;
    last_reading_.valid = false;

    status_ = DriverStatus::RUNNING;
    return true;
}

void DvlDriver::update(double dt_seconds) {
    if (status_ != DriverStatus::RUNNING) return;

    DvlReading reading;
    reading.timestamp = std::chrono::steady_clock::now();

    // Simulate bottom lock loss based on altitude
    bool bottom_lock = (true_altitude_ >= config_.min_altitude &&
                        true_altitude_ <= config_.max_altitude);
    reading.bottom_lock = bottom_lock;

    if (bottom_lock) {
        // Apply noise to velocity measurements
        reading.velocity.x = vel_noise_x_->apply(true_velocity_.x, dt_seconds);
        reading.velocity.y = vel_noise_y_->apply(true_velocity_.y, dt_seconds);
        reading.velocity.z = vel_noise_z_->apply(true_velocity_.z, dt_seconds);

        // Apply noise to altitude measurement
        reading.altitude = alt_noise_->apply(true_altitude_, dt_seconds);
        reading.valid = true;
    } else {
        // No bottom lock: output zeros with invalid flag
        reading.velocity = Vector3(0.0, 0.0, 0.0);
        reading.altitude = 0.0;
        reading.valid = false;
    }

    last_reading_ = reading;
}

void DvlDriver::shutdown() {
    status_ = DriverStatus::SHUTDOWN;
    last_reading_.valid = false;
}

std::chrono::steady_clock::time_point DvlDriver::get_last_timestamp() const {
    return last_reading_.timestamp;
}

bool DvlDriver::has_valid_reading() const {
    return last_reading_.valid && last_reading_.bottom_lock;
}

void DvlDriver::set_ground_truth(const Vector3& velocity, double altitude) {
    true_velocity_ = velocity;
    true_altitude_ = altitude;
}

}  // namespace submarine_drivers
