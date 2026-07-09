// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/core/noise_model.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <numeric>
#include <vector>

using namespace submarine_drivers;

class NoiseModelTest : public ::testing::Test {
protected:
    static constexpr unsigned int SEED = 42;
    static constexpr int NUM_SAMPLES = 10000;
};

TEST_F(NoiseModelTest, NoNoisePassthrough) {
    auto model = NoiseModel::no_noise();

    double value = 42.0;
    double result = model.apply(value, 0.01);
    EXPECT_DOUBLE_EQ(result, value);
}

TEST_F(NoiseModelTest, GaussianNoiseHasZeroMean) {
    NoiseModel::Config config;
    config.gaussian_enabled = true;
    config.gaussian_stddev = 1.0;
    config.bias_drift_enabled = false;
    config.quantization_enabled = false;

    NoiseModel model(config, SEED);

    std::vector<double> errors;
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        double noisy = model.apply(0.0, 0.01);
        errors.push_back(noisy);
    }

    double mean = std::accumulate(errors.begin(), errors.end(), 0.0) /
                  static_cast<double>(NUM_SAMPLES);

    // Mean should be close to zero (within 3 sigma / sqrt(N))
    EXPECT_NEAR(mean, 0.0, 3.0 * 1.0 / std::sqrt(NUM_SAMPLES));
}

TEST_F(NoiseModelTest, GaussianNoiseStdDev) {
    double expected_stddev = 0.5;
    NoiseModel::Config config;
    config.gaussian_enabled = true;
    config.gaussian_stddev = expected_stddev;
    config.bias_drift_enabled = false;
    config.quantization_enabled = false;

    NoiseModel model(config, SEED);

    std::vector<double> values;
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        values.push_back(model.apply(0.0, 0.01));
    }

    double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                  static_cast<double>(NUM_SAMPLES);
    double variance = 0.0;
    for (double v : values) {
        variance += (v - mean) * (v - mean);
    }
    variance /= static_cast<double>(NUM_SAMPLES - 1);
    double stddev = std::sqrt(variance);

    // Standard deviation should be close to configured value
    EXPECT_NEAR(stddev, expected_stddev, 0.05);
}

TEST_F(NoiseModelTest, QuantizationSnapsToSteps) {
    NoiseModel::Config config;
    config.gaussian_enabled = false;
    config.bias_drift_enabled = false;
    config.quantization_enabled = true;
    config.quantization_step = 0.1;

    NoiseModel model(config, SEED);

    EXPECT_DOUBLE_EQ(model.apply(0.16, 0.01), 0.2);
    EXPECT_DOUBLE_EQ(model.apply(0.14, 0.01), 0.1);
    EXPECT_DOUBLE_EQ(model.apply(0.06, 0.01), 0.1);
    EXPECT_DOUBLE_EQ(model.apply(0.04, 0.01), 0.0);
}

TEST_F(NoiseModelTest, BiasDriftAccumulates) {
    NoiseModel::Config config;
    config.gaussian_enabled = false;
    config.bias_drift_enabled = true;
    config.bias_drift_rate = 0.1;
    config.bias_drift_max = 10.0;
    config.quantization_enabled = false;

    NoiseModel model(config, SEED);

    // After many updates, bias should have drifted from zero
    for (int i = 0; i < 1000; ++i) {
        model.apply(0.0, 0.01);
    }

    EXPECT_NE(model.get_current_bias(), 0.0);
}

TEST_F(NoiseModelTest, BiasDriftClampedToMax) {
    NoiseModel::Config config;
    config.gaussian_enabled = false;
    config.bias_drift_enabled = true;
    config.bias_drift_rate = 10.0;  // very high drift rate
    config.bias_drift_max = 0.5;    // tight clamp
    config.quantization_enabled = false;

    NoiseModel model(config, SEED);

    for (int i = 0; i < 10000; ++i) {
        model.apply(0.0, 0.1);
    }

    EXPECT_LE(std::abs(model.get_current_bias()), config.bias_drift_max + 1e-10);
}

TEST_F(NoiseModelTest, ResetBiasClearsAccumulation) {
    NoiseModel::Config config;
    config.gaussian_enabled = false;
    config.bias_drift_enabled = true;
    config.bias_drift_rate = 1.0;
    config.bias_drift_max = 10.0;
    config.quantization_enabled = false;

    NoiseModel model(config, SEED);

    for (int i = 0; i < 100; ++i) {
        model.apply(0.0, 0.01);
    }
    EXPECT_NE(model.get_current_bias(), 0.0);

    model.reset_bias();
    EXPECT_DOUBLE_EQ(model.get_current_bias(), 0.0);
}

TEST_F(NoiseModelTest, VectorNoiseAppliesIndependently) {
    NoiseModel::Config config;
    config.gaussian_enabled = true;
    config.gaussian_stddev = 1.0;
    config.bias_drift_enabled = false;
    config.quantization_enabled = false;

    NoiseModel model(config, SEED);

    double x = 1.0, y = 1.0, z = 1.0;
    model.apply_vector(x, y, z, 0.01);

    // Each axis should get different noise (extremely unlikely to be equal)
    bool all_same = (x == y) && (y == z);
    EXPECT_FALSE(all_same);
}
