// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/sensors/imu_driver.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace submarine_drivers;

class ImuDriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.update_rate_hz = 200.0;
        config_.accel_noise.gaussian_enabled = false;
        config_.accel_noise.bias_drift_enabled = false;
        config_.gyro_noise.gaussian_enabled = false;
        config_.gyro_noise.bias_drift_enabled = false;

        imu_ = std::make_unique<ImuDriver>(config_);
        imu_->initialize();
    }

    ImuDriver::Config config_;
    std::unique_ptr<ImuDriver> imu_;
};

TEST_F(ImuDriverTest, InitializesCorrectly) {
    EXPECT_TRUE(imu_->is_healthy());
    EXPECT_EQ(imu_->get_status(), DriverStatus::RUNNING);
    EXPECT_EQ(imu_->get_name(), "imu_driver");
}

TEST_F(ImuDriverTest, ReturnsGravityAtRest) {
    // At rest, accelerometer should read gravity on z-axis
    imu_->set_ground_truth(Vector3(0.0, 0.0, 9.80665),
                           Vector3(0.0, 0.0, 0.0));
    imu_->update(0.005);

    auto reading = imu_->get_reading();
    EXPECT_TRUE(reading.valid);
    EXPECT_NEAR(reading.acceleration.z, 9.80665, 0.001);
    EXPECT_NEAR(reading.acceleration.x, 0.0, 0.001);
    EXPECT_NEAR(reading.angular_velocity.x, 0.0, 0.001);
}

TEST_F(ImuDriverTest, ReportsAngularVelocity) {
    Vector3 angular_vel(0.1, -0.2, 0.3);
    imu_->set_ground_truth(Vector3(0.0, 0.0, 9.80665), angular_vel);
    imu_->update(0.005);

    auto reading = imu_->get_reading();
    EXPECT_NEAR(reading.angular_velocity.x, 0.1, 0.001);
    EXPECT_NEAR(reading.angular_velocity.y, -0.2, 0.001);
    EXPECT_NEAR(reading.angular_velocity.z, 0.3, 0.001);
}

TEST_F(ImuDriverTest, NoiseAddsVariability) {
    ImuDriver::Config noisy_config;
    noisy_config.accel_noise.gaussian_enabled = true;
    noisy_config.accel_noise.gaussian_stddev = 0.5;
    noisy_config.gyro_noise.gaussian_enabled = false;

    auto noisy_imu = std::make_unique<ImuDriver>(noisy_config);
    noisy_imu->initialize();
    noisy_imu->set_ground_truth(Vector3(0.0, 0.0, 9.80665),
                                 Vector3(0.0, 0.0, 0.0));

    // Take multiple readings and check they're not all identical
    double first_z = 0.0;
    bool found_different = false;
    for (int i = 0; i < 100; ++i) {
        noisy_imu->update(0.005);
        auto reading = noisy_imu->get_reading();
        if (i == 0) {
            first_z = reading.acceleration.z;
        } else if (std::abs(reading.acceleration.z - first_z) > 0.001) {
            found_different = true;
            break;
        }
    }
    EXPECT_TRUE(found_different);
}

TEST_F(ImuDriverTest, ShutdownInvalidatesReadings) {
    imu_->set_ground_truth(Vector3(0.0, 0.0, 9.80665),
                           Vector3(0.0, 0.0, 0.0));
    imu_->update(0.005);
    EXPECT_TRUE(imu_->has_valid_reading());

    imu_->shutdown();
    EXPECT_FALSE(imu_->has_valid_reading());
    EXPECT_EQ(imu_->get_status(), DriverStatus::SHUTDOWN);
}

TEST_F(ImuDriverTest, CalibrationOffsetApplied) {
    config_.accel_bias_offset = Vector3(0.1, -0.1, 0.05);
    auto calibrated_imu = std::make_unique<ImuDriver>(config_);
    calibrated_imu->initialize();

    calibrated_imu->set_ground_truth(Vector3(1.0, 2.0, 3.0),
                                      Vector3(0.0, 0.0, 0.0));
    calibrated_imu->update(0.005);

    auto reading = calibrated_imu->get_reading();
    EXPECT_NEAR(reading.acceleration.x, 1.1, 0.001);
    EXPECT_NEAR(reading.acceleration.y, 1.9, 0.001);
    EXPECT_NEAR(reading.acceleration.z, 3.05, 0.001);
}
