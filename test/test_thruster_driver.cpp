// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/actuators/thruster_driver.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace submarine_drivers;

class ThrusterDriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.thruster_id = "T1";
        config_.pwm_neutral = 1500.0;
        config_.pwm_min = 1100.0;
        config_.pwm_max = 1900.0;
        config_.deadband_width = 25.0;
        config_.max_thrust = 50.0;
        config_.max_rpm = 3800.0;

        thruster_ = std::make_unique<ThrusterDriver>(config_);
        thruster_->initialize();
    }

    ThrusterDriver::Config config_;
    std::unique_ptr<ThrusterDriver> thruster_;
};

TEST_F(ThrusterDriverTest, InitializesToSafeState) {
    EXPECT_TRUE(thruster_->is_healthy());
    EXPECT_TRUE(thruster_->is_in_safe_state());
    EXPECT_FALSE(thruster_->is_armed());
    EXPECT_EQ(thruster_->get_status(), DriverStatus::RUNNING);
}

TEST_F(ThrusterDriverTest, DisarmedThrustersProduceZeroForce) {
    // Not armed — command should be rejected
    auto cmd = thruster_->command_force(25.0);
    EXPECT_DOUBLE_EQ(cmd.force_newtons, 0.0);
    EXPECT_DOUBLE_EQ(cmd.pwm_microseconds, config_.pwm_neutral);
}

TEST_F(ThrusterDriverTest, ArmedThrusterAcceptsCommands) {
    thruster_->arm();
    EXPECT_TRUE(thruster_->is_armed());

    auto cmd = thruster_->command_force(25.0);
    EXPECT_GT(cmd.force_newtons, 0.0);
    EXPECT_GT(cmd.pwm_microseconds, config_.pwm_neutral);
}

TEST_F(ThrusterDriverTest, ForceClampedToMax) {
    thruster_->arm();

    auto cmd = thruster_->command_force(100.0);  // exceeds max
    EXPECT_LE(cmd.force_newtons, config_.max_thrust);
}

TEST_F(ThrusterDriverTest, NegativeForceReverses) {
    thruster_->arm();

    auto cmd = thruster_->command_force(-25.0);
    EXPECT_LT(cmd.force_newtons, 0.0);
    EXPECT_LT(cmd.pwm_microseconds, config_.pwm_neutral);
}

TEST_F(ThrusterDriverTest, DeadbandProducesZero) {
    // PWM within deadband should produce zero force
    double force = thruster_->pwm_to_force(config_.pwm_neutral);
    EXPECT_DOUBLE_EQ(force, 0.0);

    force = thruster_->pwm_to_force(config_.pwm_neutral + 10.0);  // within deadband
    EXPECT_DOUBLE_EQ(force, 0.0);

    force = thruster_->pwm_to_force(config_.pwm_neutral - 10.0);
    EXPECT_DOUBLE_EQ(force, 0.0);
}

TEST_F(ThrusterDriverTest, OutsideDeadbandProducesForce) {
    double pwm = config_.pwm_neutral + config_.deadband_width + 50.0;
    double force = thruster_->pwm_to_force(pwm);
    EXPECT_GT(force, 0.0);

    pwm = config_.pwm_neutral - config_.deadband_width - 50.0;
    force = thruster_->pwm_to_force(pwm);
    EXPECT_LT(force, 0.0);
}

TEST_F(ThrusterDriverTest, QuadraticThrustCurve) {
    // Force should scale quadratically: doubling PWM deviation
    // should roughly quadruple force
    double pwm_small = config_.pwm_neutral + config_.deadband_width + 50.0;
    double pwm_large = config_.pwm_neutral + config_.deadband_width + 100.0;

    double force_small = thruster_->pwm_to_force(pwm_small);
    double force_large = thruster_->pwm_to_force(pwm_large);

    // With quadratic curve and double the deviation, expect ~4x force
    double deviation_ratio = 100.0 / 50.0;
    double expected_ratio = deviation_ratio * deviation_ratio;
    double actual_ratio = force_large / force_small;

    EXPECT_NEAR(actual_ratio, expected_ratio, 0.5);
}

TEST_F(ThrusterDriverTest, ForceToAndFromPwmRoundTrip) {
    thruster_->arm();

    double original_force = 20.0;
    double pwm = thruster_->force_to_pwm(original_force);
    double recovered_force = thruster_->pwm_to_force(pwm);

    EXPECT_NEAR(recovered_force, original_force, 0.5);
}

TEST_F(ThrusterDriverTest, DisarmSetsSafeState) {
    thruster_->arm();
    thruster_->command_force(25.0);
    EXPECT_TRUE(thruster_->is_armed());

    thruster_->disarm();
    EXPECT_FALSE(thruster_->is_armed());
    EXPECT_TRUE(thruster_->is_in_safe_state());
}

TEST_F(ThrusterDriverTest, ShutdownDisablesDriver) {
    thruster_->arm();
    thruster_->command_force(25.0);

    thruster_->shutdown();
    EXPECT_EQ(thruster_->get_status(), DriverStatus::SHUTDOWN);
    EXPECT_TRUE(thruster_->is_in_safe_state());
}

TEST_F(ThrusterDriverTest, MaxOutputReturnsConfiguredMax) {
    EXPECT_DOUBLE_EQ(thruster_->get_max_output(), config_.max_thrust);
}
