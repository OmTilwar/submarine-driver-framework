// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/actuators/thruster_driver.hpp"
#include <algorithm>
#include <cmath>

namespace submarine_drivers {

ThrusterDriver::ThrusterDriver(const Config& config)
    : ActuatorBase("thruster_" + config.thruster_id)
    , config_(config)
    , current_temperature_(config.ambient_temperature) {}

bool ThrusterDriver::initialize() {
    status_ = DriverStatus::INITIALIZING;

    last_command_ = ThrusterCommand{0.0, config_.pwm_neutral};
    feedback_ = ThrusterFeedback{0.0, 0.0, config_.ambient_temperature, true};
    current_temperature_ = config_.ambient_temperature;
    armed_ = false;

    status_ = DriverStatus::RUNNING;
    return true;
}

void ThrusterDriver::update(double dt_seconds) {
    if (status_ != DriverStatus::RUNNING) return;

    // Calculate power fraction for thermal model
    double power_fraction = std::abs(last_command_.force_newtons) /
                            config_.max_thrust;
    power_fraction = std::clamp(power_fraction, 0.0, 1.0);

    // Update thermal model
    update_thermal(dt_seconds, power_fraction);

    // Check thermal protection
    if (current_temperature_ >= config_.max_temperature) {
        status_ = DriverStatus::WARNING;
        set_safe_state();
        return;
    }

    // Update feedback
    feedback_.actual_force = last_command_.force_newtons;
    feedback_.current_draw = power_fraction * 20.0;  // simplified: max 20A
    feedback_.temperature = current_temperature_;
    feedback_.healthy = (status_ == DriverStatus::RUNNING);
}

void ThrusterDriver::shutdown() {
    set_safe_state();
    status_ = DriverStatus::SHUTDOWN;
}

void ThrusterDriver::set_safe_state() {
    last_command_ = ThrusterCommand{0.0, config_.pwm_neutral};
    feedback_.actual_force = 0.0;
    armed_ = false;
}

bool ThrusterDriver::is_in_safe_state() const {
    return !armed_ &&
           std::abs(last_command_.force_newtons) < 0.01 &&
           std::abs(last_command_.pwm_microseconds - config_.pwm_neutral) < 1.0;
}

double ThrusterDriver::get_max_output() const {
    return config_.max_thrust;
}

ThrusterCommand ThrusterDriver::command_force(double force_newtons) {
    if (!armed_ || status_ != DriverStatus::RUNNING) {
        return ThrusterCommand{0.0, config_.pwm_neutral};
    }

    // Clamp force
    force_newtons = std::clamp(force_newtons, -config_.max_thrust,
                                config_.max_thrust);

    ThrusterCommand cmd;
    cmd.force_newtons = force_newtons;
    cmd.pwm_microseconds = force_to_pwm(force_newtons);

    last_command_ = cmd;
    return cmd;
}

ThrusterCommand ThrusterDriver::command_pwm(double pwm_us) {
    if (!armed_ || status_ != DriverStatus::RUNNING) {
        return ThrusterCommand{0.0, config_.pwm_neutral};
    }

    // Clamp PWM
    pwm_us = std::clamp(pwm_us, config_.pwm_min, config_.pwm_max);

    // Apply deadband
    pwm_us = apply_deadband(pwm_us);

    ThrusterCommand cmd;
    cmd.pwm_microseconds = pwm_us;
    cmd.force_newtons = pwm_to_force(pwm_us);

    last_command_ = cmd;
    return cmd;
}

double ThrusterDriver::pwm_to_force(double pwm_us) const {
    double deviation = pwm_us - config_.pwm_neutral;

    // Inside deadband → zero force
    if (std::abs(deviation) <= config_.deadband_width) {
        return 0.0;
    }

    // Normalize to [-1, 1] range
    double range = (deviation > 0)
        ? (config_.pwm_max - config_.pwm_neutral - config_.deadband_width)
        : (config_.pwm_neutral - config_.pwm_min - config_.deadband_width);

    if (range <= 0.0) return 0.0;

    double sign = (deviation > 0) ? 1.0 : -1.0;
    double magnitude = (std::abs(deviation) - config_.deadband_width) / range;
    magnitude = std::clamp(magnitude, 0.0, 1.0);

    // Quadratic thrust curve: F = k * |normalized|^2 * sign * max_thrust
    double force = sign * config_.max_thrust * magnitude * magnitude;

    return std::clamp(force, -config_.max_thrust, config_.max_thrust);
}

double ThrusterDriver::force_to_pwm(double force_newtons) const {
    if (std::abs(force_newtons) < 0.01) {
        return config_.pwm_neutral;
    }

    double sign = (force_newtons > 0) ? 1.0 : -1.0;
    double magnitude = std::sqrt(std::abs(force_newtons) / config_.max_thrust);
    magnitude = std::clamp(magnitude, 0.0, 1.0);

    double range = (sign > 0)
        ? (config_.pwm_max - config_.pwm_neutral - config_.deadband_width)
        : (config_.pwm_neutral - config_.pwm_min - config_.deadband_width);

    return config_.pwm_neutral + sign * (config_.deadband_width + magnitude * range);
}

void ThrusterDriver::disarm() {
    set_safe_state();
}

double ThrusterDriver::apply_deadband(double pwm_us) const {
    double deviation = pwm_us - config_.pwm_neutral;

    if (std::abs(deviation) <= config_.deadband_width) {
        return config_.pwm_neutral;  // snap to neutral inside deadband
    }

    return pwm_us;
}

void ThrusterDriver::update_thermal(double dt_seconds, double power_fraction) {
    // Simple first-order thermal model:
    // dT/dt = (T_target - T_current) / tau
    // T_target = T_ambient + power_fraction * (T_max - T_ambient)
    double target_temp = config_.ambient_temperature +
                         power_fraction *
                         (config_.max_temperature - config_.ambient_temperature);

    double alpha = dt_seconds / config_.thermal_time_constant;
    alpha = std::clamp(alpha, 0.0, 1.0);

    current_temperature_ += alpha * (target_temp - current_temperature_);
}

}  // namespace submarine_drivers
