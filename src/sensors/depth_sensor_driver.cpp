// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/sensors/depth_sensor_driver.hpp"

namespace submarine_drivers {

DepthSensorDriver::DepthSensorDriver(const Config& config)
    : SensorBase("depth_sensor", config.update_rate_hz)
    , config_(config) {}

bool DepthSensorDriver::initialize() {
    status_ = DriverStatus::INITIALIZING;

    pressure_noise_ = std::make_unique<NoiseModel>(config_.pressure_noise);
    temperature_noise_ = std::make_unique<NoiseModel>(config_.temperature_noise);

    true_depth_ = 0.0;
    true_temperature_ = config_.reference_temp;
    last_reading_.valid = false;

    status_ = DriverStatus::RUNNING;
    return true;
}

void DepthSensorDriver::update(double dt_seconds) {
    if (status_ != DriverStatus::RUNNING) return;

    DepthReading reading;
    reading.timestamp = std::chrono::steady_clock::now();

    // Convert ground-truth depth to pressure
    double rho = compensated_density(true_temperature_);
    double true_pressure = config_.atmospheric_pressure +
                           rho * config_.gravity * true_depth_;

    // Apply noise to raw pressure and temperature
    reading.pressure = pressure_noise_->apply(true_pressure, dt_seconds);
    reading.temperature = temperature_noise_->apply(true_temperature_, dt_seconds);

    // Convert noisy pressure back to depth (this is what the real sensor does)
    reading.depth = pressure_to_depth(reading.pressure, reading.temperature);
    reading.valid = (reading.depth >= 0.0);

    last_reading_ = reading;
}

void DepthSensorDriver::shutdown() {
    status_ = DriverStatus::SHUTDOWN;
    last_reading_.valid = false;
}

std::chrono::steady_clock::time_point DepthSensorDriver::get_last_timestamp() const {
    return last_reading_.timestamp;
}

bool DepthSensorDriver::has_valid_reading() const {
    return last_reading_.valid;
}

void DepthSensorDriver::set_ground_truth(double depth_meters,
                                          double temperature_celsius) {
    true_depth_ = depth_meters;
    true_temperature_ = temperature_celsius;
}

double DepthSensorDriver::pressure_to_depth(double pressure_pa,
                                              double temperature_c) const {
    double rho = compensated_density(temperature_c);
    double gauge_pressure = pressure_pa - config_.atmospheric_pressure;

    if (rho * config_.gravity <= 0.0) return 0.0;

    return gauge_pressure / (rho * config_.gravity);
}

double DepthSensorDriver::compensated_density(double temperature_c) const {
    // Linear temperature compensation model
    double delta_temp = temperature_c - config_.reference_temp;
    return config_.water_density + config_.density_temp_coeff * delta_temp;
}

}  // namespace submarine_drivers
