// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/sensors/depth_sensor_driver.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace submarine_drivers;

class DepthSensorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.update_rate_hz = 10.0;
        config_.atmospheric_pressure = 101325.0;
        config_.water_density = 1025.0;
        config_.gravity = 9.80665;
        config_.reference_temp = 15.0;
        config_.density_temp_coeff = -0.2;
        config_.pressure_noise.gaussian_enabled = false;
        config_.temperature_noise.gaussian_enabled = false;

        sensor_ = std::make_unique<DepthSensorDriver>(config_);
        sensor_->initialize();
    }

    DepthSensorDriver::Config config_;
    std::unique_ptr<DepthSensorDriver> sensor_;
};

TEST_F(DepthSensorTest, InitializesCorrectly) {
    EXPECT_TRUE(sensor_->is_healthy());
    EXPECT_EQ(sensor_->get_name(), "depth_sensor");
}

TEST_F(DepthSensorTest, ZeroDepthAtSurface) {
    sensor_->set_ground_truth(0.0, 15.0);
    sensor_->update(0.1);

    auto reading = sensor_->get_reading();
    EXPECT_TRUE(reading.valid);
    EXPECT_NEAR(reading.depth, 0.0, 0.01);
}

TEST_F(DepthSensorTest, CorrectDepthAt10Meters) {
    sensor_->set_ground_truth(10.0, 15.0);
    sensor_->update(0.1);

    auto reading = sensor_->get_reading();
    EXPECT_TRUE(reading.valid);
    EXPECT_NEAR(reading.depth, 10.0, 0.1);
}

TEST_F(DepthSensorTest, CorrectDepthAt100Meters) {
    sensor_->set_ground_truth(100.0, 15.0);
    sensor_->update(0.1);

    auto reading = sensor_->get_reading();
    EXPECT_NEAR(reading.depth, 100.0, 0.5);
}

TEST_F(DepthSensorTest, PressureIncreasesWithDepth) {
    sensor_->set_ground_truth(0.0, 15.0);
    sensor_->update(0.1);
    double pressure_surface = sensor_->get_reading().pressure;

    sensor_->set_ground_truth(50.0, 15.0);
    sensor_->update(0.1);
    double pressure_deep = sensor_->get_reading().pressure;

    EXPECT_GT(pressure_deep, pressure_surface);
}

TEST_F(DepthSensorTest, TemperatureCompensation) {
    // At reference temp, density = 1025 kg/m^3
    double depth_ref = sensor_->pressure_to_depth(
        101325.0 + 1025.0 * 9.80665 * 10.0, 15.0);
    EXPECT_NEAR(depth_ref, 10.0, 0.01);

    // At higher temp, density decreases → same pressure = deeper reading
    double depth_warm = sensor_->pressure_to_depth(
        101325.0 + 1025.0 * 9.80665 * 10.0, 25.0);
    // With lower density, same gauge pressure indicates slightly greater depth
    EXPECT_GT(depth_warm, depth_ref);
}

TEST_F(DepthSensorTest, ShutdownInvalidatesReading) {
    sensor_->set_ground_truth(10.0, 15.0);
    sensor_->update(0.1);
    EXPECT_TRUE(sensor_->has_valid_reading());

    sensor_->shutdown();
    EXPECT_FALSE(sensor_->has_valid_reading());
}
