// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/core/noise_model.hpp"
#include <algorithm>

namespace submarine_drivers {

NoiseModel::NoiseModel()
    : config_{}
    , rng_(std::random_device{}())
    , gaussian_dist_(0.0, config_.gaussian_stddev)
    , bias_walk_dist_(0.0, 1.0) {}

NoiseModel::NoiseModel(const Config& config)
    : config_(config)
    , rng_(std::random_device{}())
    , gaussian_dist_(0.0, config.gaussian_stddev)
    , bias_walk_dist_(0.0, 1.0) {}

NoiseModel::NoiseModel(const Config& config, unsigned int seed)
    : config_(config)
    , rng_(seed)
    , gaussian_dist_(0.0, config.gaussian_stddev)
    , bias_walk_dist_(0.0, 1.0) {}

double NoiseModel::apply(double clean_value, double dt_seconds) {
    double noisy_value = clean_value;

    if (config_.gaussian_enabled) {
        noisy_value = apply_gaussian(noisy_value);
    }

    if (config_.bias_drift_enabled) {
        noisy_value = apply_bias_drift(noisy_value, dt_seconds);
    }

    if (config_.quantization_enabled) {
        noisy_value = apply_quantization(noisy_value);
    }

    return noisy_value;
}

void NoiseModel::apply_vector(double& x, double& y, double& z,
                               double dt_seconds) {
    x = apply(x, dt_seconds);
    y = apply(y, dt_seconds);
    z = apply(z, dt_seconds);
}

void NoiseModel::reset_bias() {
    current_bias_ = 0.0;
}

NoiseModel NoiseModel::no_noise() {
    Config config;
    config.gaussian_enabled = false;
    config.bias_drift_enabled = false;
    config.quantization_enabled = false;
    return NoiseModel(config);
}

double NoiseModel::apply_gaussian(double value) {
    return value + gaussian_dist_(rng_);
}

double NoiseModel::apply_bias_drift(double value, double dt_seconds) {
    // Random walk: bias changes by a small random amount each step
    double drift_increment = bias_walk_dist_(rng_) *
                             config_.bias_drift_rate *
                             std::sqrt(dt_seconds);
    current_bias_ += drift_increment;

    // Clamp bias to maximum
    current_bias_ = std::clamp(current_bias_,
                               -config_.bias_drift_max,
                                config_.bias_drift_max);

    return value + current_bias_;
}

double NoiseModel::apply_quantization(double value) {
    if (config_.quantization_step <= 0.0) {
        return value;
    }
    return std::round(value / config_.quantization_step) *
           config_.quantization_step;
}

}  // namespace submarine_drivers
