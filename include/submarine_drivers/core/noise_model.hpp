// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include <cmath>
#include <random>

namespace submarine_drivers {

/**
 * @brief Configurable noise injection for software-in-the-loop (SIL) testing.
 *
 * Models realistic sensor degradation with three noise components:
 *   1. Gaussian white noise (zero-mean, configurable std deviation)
 *   2. Bias drift (random walk, simulates sensor aging/temperature effects)
 *   3. Quantization noise (simulates ADC resolution limits)
 *
 * Each component can be independently enabled/disabled and configured.
 * This enables testing autonomy algorithms against realistic sensor
 * degradation without requiring physical hardware.
 */
class NoiseModel {
public:
    struct Config {
        // Gaussian white noise
        bool gaussian_enabled{true};
        double gaussian_stddev{0.01};

        // Bias drift (random walk)
        bool bias_drift_enabled{false};
        double bias_drift_rate{0.001};    // drift rate per second
        double bias_drift_max{0.1};       // maximum accumulated bias

        // Quantization
        bool quantization_enabled{false};
        double quantization_step{0.001};  // minimum resolvable change
    };

    /// Construct with default config (Gaussian noise only)
    NoiseModel();

    /// Construct with custom config
    explicit NoiseModel(const Config& config);

    /// Construct with a specific random seed (for reproducible tests)
    NoiseModel(const Config& config, unsigned int seed);

    /// Apply all enabled noise components to a clean value
    double apply(double clean_value, double dt_seconds);

    /// Apply noise to a 3D vector (independent noise per axis)
    void apply_vector(double& x, double& y, double& z, double dt_seconds);

    /// Reset accumulated bias drift to zero
    void reset_bias();

    /// Get current accumulated bias
    double get_current_bias() const { return current_bias_; }

    /// Get the configuration
    const Config& get_config() const { return config_; }

    /// Update configuration at runtime
    void set_config(const Config& config) { config_ = config; }

    /// Create a noise model with no noise (passthrough)
    static NoiseModel no_noise();

private:
    Config config_;
    std::mt19937 rng_;
    std::normal_distribution<double> gaussian_dist_;
    std::normal_distribution<double> bias_walk_dist_;
    double current_bias_{0.0};

    double apply_gaussian(double value);
    double apply_bias_drift(double value, double dt_seconds);
    double apply_quantization(double value);
};

}  // namespace submarine_drivers
