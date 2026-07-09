// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/sensors/dvl_driver.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace submarine_drivers;

class DvlDriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.update_rate_hz = 5.0;
        config_.max_altitude = 200.0;
        config_.min_altitude = 0.3;
        config_.velocity_noise.gaussian_enabled = false;
        config_.altitude_noise.gaussian_enabled = false;

        dvl_ = std::make_unique<DvlDriver>(config_);
        dvl_->initialize();
    }

    DvlDriver::Config config_;
    std::unique_ptr<DvlDriver> dvl_;
};

TEST_F(DvlDriverTest, InitializesCorrectly) {
    EXPECT_TRUE(dvl_->is_healthy());
    EXPECT_EQ(dvl_->get_name(), "dvl_driver");
}

TEST_F(DvlDriverTest, ReportsVelocityWithBottomLock) {
    dvl_->set_ground_truth(Vector3(1.5, -0.3, 0.1), 50.0);
    dvl_->update(0.2);

    auto reading = dvl_->get_reading();
    EXPECT_TRUE(reading.valid);
    EXPECT_TRUE(reading.bottom_lock);
    EXPECT_NEAR(reading.velocity.x, 1.5, 0.001);
    EXPECT_NEAR(reading.velocity.y, -0.3, 0.001);
    EXPECT_NEAR(reading.altitude, 50.0, 0.001);
}

TEST_F(DvlDriverTest, LosesBottomLockAtHighAltitude) {
    dvl_->set_ground_truth(Vector3(1.0, 0.0, 0.0), 300.0);  // above max
    dvl_->update(0.2);

    auto reading = dvl_->get_reading();
    EXPECT_FALSE(reading.bottom_lock);
    EXPECT_FALSE(reading.valid);
    EXPECT_FALSE(dvl_->has_bottom_lock());
}

TEST_F(DvlDriverTest, LosesBottomLockTooClose) {
    dvl_->set_ground_truth(Vector3(1.0, 0.0, 0.0), 0.1);  // below min
    dvl_->update(0.2);

    auto reading = dvl_->get_reading();
    EXPECT_FALSE(reading.bottom_lock);
    EXPECT_FALSE(reading.valid);
}

TEST_F(DvlDriverTest, RegainsBottomLock) {
    // Lose bottom lock
    dvl_->set_ground_truth(Vector3(1.0, 0.0, 0.0), 300.0);
    dvl_->update(0.2);
    EXPECT_FALSE(dvl_->has_bottom_lock());

    // Regain bottom lock
    dvl_->set_ground_truth(Vector3(1.0, 0.0, 0.0), 100.0);
    dvl_->update(0.2);
    EXPECT_TRUE(dvl_->has_bottom_lock());
    EXPECT_TRUE(dvl_->has_valid_reading());
}

TEST_F(DvlDriverTest, ShutdownInvalidatesReading) {
    dvl_->set_ground_truth(Vector3(1.0, 0.0, 0.0), 50.0);
    dvl_->update(0.2);
    EXPECT_TRUE(dvl_->has_valid_reading());

    dvl_->shutdown();
    EXPECT_FALSE(dvl_->has_valid_reading());
}
