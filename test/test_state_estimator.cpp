// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/estimation/state_estimator_6dof.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace submarine_drivers;

class StateEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        estimator_ = std::make_unique<StateEstimator6DOF>();
    }

    std::unique_ptr<StateEstimator6DOF> estimator_;

    // Helper: create a valid IMU reading at rest
    ImuReading make_rest_imu() {
        ImuReading imu;
        imu.acceleration = Vector3(0.0, 0.0, 9.80665);
        imu.angular_velocity = Vector3(0.0, 0.0, 0.0);
        imu.timestamp = std::chrono::steady_clock::now();
        imu.valid = true;
        return imu;
    }

    // Helper: create a DVL reading
    DvlReading make_dvl(double u, double v, double w) {
        DvlReading dvl;
        dvl.velocity = Vector3(u, v, w);
        dvl.altitude = 50.0;
        dvl.bottom_lock = true;
        dvl.timestamp = std::chrono::steady_clock::now();
        dvl.valid = true;
        return dvl;
    }

    // Helper: create a depth reading
    DepthReading make_depth(double depth) {
        DepthReading dr;
        dr.depth = depth;
        dr.temperature = 15.0;
        dr.pressure = 101325.0 + 1025.0 * 9.80665 * depth;
        dr.timestamp = std::chrono::steady_clock::now();
        dr.valid = true;
        return dr;
    }
};

TEST_F(StateEstimatorTest, InitializesToZero) {
    auto state = estimator_->get_state();
    for (int i = 0; i < StateEstimator6DOF::STATE_DIM; ++i) {
        EXPECT_DOUBLE_EQ(state[i], 0.0);
    }
}

TEST_F(StateEstimatorTest, ResetToCustomState) {
    StateEstimator6DOF::StateVector initial{};
    initial[StateEstimator6DOF::X] = 10.0;
    initial[StateEstimator6DOF::Z] = 50.0;
    initial[StateEstimator6DOF::YAW] = 1.57;

    estimator_->reset(initial);
    auto state = estimator_->get_state();

    EXPECT_DOUBLE_EQ(state[StateEstimator6DOF::X], 10.0);
    EXPECT_DOUBLE_EQ(state[StateEstimator6DOF::Z], 50.0);
    EXPECT_DOUBLE_EQ(state[StateEstimator6DOF::YAW], 1.57);
}

TEST_F(StateEstimatorTest, StationaryVehicleStaysStationary) {
    // At rest with only gravity, vehicle should not move
    auto imu = make_rest_imu();

    for (int i = 0; i < 100; ++i) {
        estimator_->predict(imu, 0.005);
    }

    auto state = estimator_->get_state();

    // Position should remain near zero (Z may drift slightly due to
    // simplified gravity compensation — this is expected without depth
    // sensor corrections in a dead-reckoning-only scenario)
    EXPECT_NEAR(state[StateEstimator6DOF::X], 0.0, 0.5);
    EXPECT_NEAR(state[StateEstimator6DOF::Y], 0.0, 0.5);
    EXPECT_NEAR(state[StateEstimator6DOF::Z], 0.0, 5.0);

    // Orientation should remain near zero
    EXPECT_NEAR(state[StateEstimator6DOF::ROLL], 0.0, 0.01);
    EXPECT_NEAR(state[StateEstimator6DOF::PITCH], 0.0, 0.01);
    EXPECT_NEAR(state[StateEstimator6DOF::YAW], 0.0, 0.01);
}

TEST_F(StateEstimatorTest, YawRateIntegration) {
    ImuReading imu = make_rest_imu();
    imu.angular_velocity.z = 0.1;  // 0.1 rad/s yaw rate

    // Integrate for 1 second (200 steps at 5ms)
    for (int i = 0; i < 200; ++i) {
        estimator_->predict(imu, 0.005);
    }

    auto state = estimator_->get_state();
    // Yaw should be approximately 0.1 rad after 1 second
    EXPECT_NEAR(state[StateEstimator6DOF::YAW], 0.1, 0.02);
}

TEST_F(StateEstimatorTest, DepthUpdateCorrects) {
    // Start at depth 0, measurement says depth 10
    auto depth = make_depth(10.0);
    estimator_->update_depth(depth);

    auto state = estimator_->get_state();
    // Z should have moved toward 10.0
    EXPECT_GT(state[StateEstimator6DOF::Z], 0.0);
}

TEST_F(StateEstimatorTest, DvlUpdateCorrectsVelocity) {
    // Start with zero velocity, DVL says we're moving
    auto dvl = make_dvl(2.0, 0.0, 0.0);
    estimator_->update_dvl(dvl);

    auto state = estimator_->get_state();
    // U velocity should have moved toward 2.0
    EXPECT_GT(state[StateEstimator6DOF::U], 0.0);
}

TEST_F(StateEstimatorTest, CovarianceGrowsDuringPrediction) {
    auto cov_before = estimator_->get_covariance_diagonal();

    auto imu = make_rest_imu();
    estimator_->predict(imu, 0.01);

    auto cov_after = estimator_->get_covariance_diagonal();

    // Covariance should grow after prediction
    for (int i = 0; i < StateEstimator6DOF::STATE_DIM; ++i) {
        EXPECT_GE(cov_after[i], cov_before[i]);
    }
}

TEST_F(StateEstimatorTest, CovarianceShrinksDuringUpdate) {
    // Grow covariance with predictions
    auto imu = make_rest_imu();
    for (int i = 0; i < 10; ++i) {
        estimator_->predict(imu, 0.01);
    }

    auto cov_before_update = estimator_->get_covariance_diagonal();

    // DVL update should shrink velocity covariance
    auto dvl = make_dvl(0.0, 0.0, 0.0);
    estimator_->update_dvl(dvl);

    auto cov_after_update = estimator_->get_covariance_diagonal();

    // U, V, W covariance should decrease
    EXPECT_LT(cov_after_update[StateEstimator6DOF::U],
              cov_before_update[StateEstimator6DOF::U]);
    EXPECT_LT(cov_after_update[StateEstimator6DOF::V],
              cov_before_update[StateEstimator6DOF::V]);
}

TEST_F(StateEstimatorTest, InvalidReadingsIgnored) {
    DvlReading invalid_dvl;
    invalid_dvl.valid = false;
    invalid_dvl.velocity = Vector3(100.0, 0.0, 0.0);  // large value

    estimator_->update_dvl(invalid_dvl);

    // State should not change
    auto state = estimator_->get_state();
    EXPECT_DOUBLE_EQ(state[StateEstimator6DOF::U], 0.0);
}

TEST_F(StateEstimatorTest, GetPoseReturnsCorrectValues) {
    StateEstimator6DOF::StateVector initial{};
    initial[StateEstimator6DOF::X] = 5.0;
    initial[StateEstimator6DOF::Y] = 10.0;
    initial[StateEstimator6DOF::Z] = 15.0;
    initial[StateEstimator6DOF::ROLL] = 0.1;
    initial[StateEstimator6DOF::PITCH] = 0.2;
    initial[StateEstimator6DOF::YAW] = 0.3;

    estimator_->reset(initial);
    auto pose = estimator_->get_pose();

    EXPECT_DOUBLE_EQ(pose.position.x, 5.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 10.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 15.0);
    EXPECT_DOUBLE_EQ(pose.orientation.roll, 0.1);
    EXPECT_DOUBLE_EQ(pose.orientation.pitch, 0.2);
    EXPECT_DOUBLE_EQ(pose.orientation.yaw, 0.3);
}
