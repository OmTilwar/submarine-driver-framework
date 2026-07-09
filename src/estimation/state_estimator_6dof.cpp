// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#include "submarine_drivers/estimation/state_estimator_6dof.hpp"
#include <cmath>
#include <algorithm>

namespace submarine_drivers {

StateEstimator6DOF::StateEstimator6DOF(const Config& config)
    : config_(config) {
    reset();
}

void StateEstimator6DOF::reset(const StateVector& initial_state) {
    state_ = initial_state;

    // Initialize covariance diagonal with high uncertainty
    covariance_diag_.fill(1.0);
    last_innovation_norm_ = 0.0;
}

void StateEstimator6DOF::predict(const ImuReading& imu, double dt_seconds) {
    if (!imu.valid || dt_seconds <= 0.0) return;

    // --- Orientation update (gyroscope integration) ---
    // Euler angle rates from body angular velocity (small angle approx)
    double sin_roll = std::sin(state_[ROLL]);
    double cos_roll = std::cos(state_[ROLL]);
    double cos_pitch = std::cos(state_[PITCH]);
    double tan_pitch = std::tan(state_[PITCH]);

    // Prevent division by zero near gimbal lock
    if (std::abs(cos_pitch) < 1e-6) {
        cos_pitch = (cos_pitch >= 0) ? 1e-6 : -1e-6;
        tan_pitch = std::sin(state_[PITCH]) / cos_pitch;
    }

    double p = imu.angular_velocity.x;
    double q = imu.angular_velocity.y;
    double r = imu.angular_velocity.z;

    // Euler kinematic equations
    double roll_dot = p + sin_roll * tan_pitch * q + cos_roll * tan_pitch * r;
    double pitch_dot = cos_roll * q - sin_roll * r;
    double yaw_dot = (sin_roll / cos_pitch) * q + (cos_roll / cos_pitch) * r;

    state_[ROLL]  += roll_dot * dt_seconds;
    state_[PITCH] += pitch_dot * dt_seconds;
    state_[YAW]   += yaw_dot * dt_seconds;

    // Normalize angles
    state_[ROLL]  = normalize_angle(state_[ROLL]);
    state_[PITCH] = normalize_angle(state_[PITCH]);
    state_[YAW]   = normalize_angle(state_[YAW]);

    // Store angular velocity
    state_[P] = p;
    state_[Q] = q;
    state_[R] = r;

    // --- Velocity update (accelerometer integration) ---
    // Remove gravity component (rotate gravity to body frame and subtract)
    double sin_pitch = std::sin(state_[PITCH]);
    cos_pitch = std::cos(state_[PITCH]);
    sin_roll = std::sin(state_[ROLL]);
    cos_roll = std::cos(state_[ROLL]);

    // Gravity in body frame (NED convention: gravity is +z in NED)
    double gx_body =  config_.gravity * sin_pitch;
    double gy_body = -config_.gravity * cos_pitch * sin_roll;
    double gz_body = -config_.gravity * cos_pitch * cos_roll;

    // Linear acceleration in body frame (minus gravity)
    double ax = imu.acceleration.x - gx_body;
    double ay = imu.acceleration.y - gy_body;
    double az = imu.acceleration.z - gz_body;

    // Coriolis/centripetal terms (omega x v)
    double cx = state_[Q] * state_[W] - state_[R] * state_[V];
    double cy = state_[R] * state_[U] - state_[P] * state_[W];
    double cz = state_[P] * state_[V] - state_[Q] * state_[U];

    // Integrate acceleration to get velocity (body frame)
    state_[U] += (ax - cx) * dt_seconds;
    state_[V] += (ay - cy) * dt_seconds;
    state_[W] += (az - cz) * dt_seconds;

    // --- Position update (velocity integration) ---
    // Rotate body velocity to NED frame
    Vector3 vel_body(state_[U], state_[V], state_[W]);
    Vector3 vel_ned = body_to_ned(vel_body);

    state_[X] += vel_ned.x * dt_seconds;
    state_[Y] += vel_ned.y * dt_seconds;
    state_[Z] += vel_ned.z * dt_seconds;

    // --- Covariance prediction (simplified: grow diagonal) ---
    double dt2 = dt_seconds * dt_seconds;
    covariance_diag_[X] += config_.position_process_noise * dt2;
    covariance_diag_[Y] += config_.position_process_noise * dt2;
    covariance_diag_[Z] += config_.position_process_noise * dt2;
    covariance_diag_[ROLL]  += config_.orientation_process_noise * dt2;
    covariance_diag_[PITCH] += config_.orientation_process_noise * dt2;
    covariance_diag_[YAW]   += config_.orientation_process_noise * dt2;
    covariance_diag_[U] += config_.velocity_process_noise * dt2;
    covariance_diag_[V] += config_.velocity_process_noise * dt2;
    covariance_diag_[W] += config_.velocity_process_noise * dt2;
    covariance_diag_[P] += config_.angular_rate_process_noise * dt2;
    covariance_diag_[Q] += config_.angular_rate_process_noise * dt2;
    covariance_diag_[R] += config_.angular_rate_process_noise * dt2;
}

void StateEstimator6DOF::update_dvl(const DvlReading& dvl) {
    if (!dvl.valid || !dvl.bottom_lock) return;

    // DVL measures body-frame velocity directly
    // Innovation: measurement - predicted
    double innov_u = dvl.velocity.x - state_[U];
    double innov_v = dvl.velocity.y - state_[V];
    double innov_w = dvl.velocity.z - state_[W];

    last_innovation_norm_ = std::sqrt(innov_u * innov_u +
                                       innov_v * innov_v +
                                       innov_w * innov_w);

    // Apply scalar updates for each velocity component
    scalar_update(U, innov_u, config_.dvl_velocity_noise);
    scalar_update(V, innov_v, config_.dvl_velocity_noise);
    scalar_update(W, innov_w, config_.dvl_velocity_noise);
}

void StateEstimator6DOF::update_depth(const DepthReading& depth) {
    if (!depth.valid) return;

    // Depth sensor measures z-position directly (positive down in NED)
    double innovation = depth.depth - state_[Z];
    last_innovation_norm_ = std::abs(innovation);

    scalar_update(Z, innovation, config_.depth_noise);
}

Pose6DOF StateEstimator6DOF::get_pose() const {
    Pose6DOF pose;
    pose.position = Vector3(state_[X], state_[Y], state_[Z]);
    pose.orientation = EulerAngles(state_[ROLL], state_[PITCH], state_[YAW]);
    pose.timestamp = std::chrono::steady_clock::now();
    return pose;
}

Velocity6DOF StateEstimator6DOF::get_velocity() const {
    Velocity6DOF vel;
    vel.linear = Vector3(state_[U], state_[V], state_[W]);
    vel.angular = Vector3(state_[P], state_[Q], state_[R]);
    return vel;
}

Vector3 StateEstimator6DOF::body_to_ned(const Vector3& body_vec) const {
    double cr = std::cos(state_[ROLL]);
    double sr = std::sin(state_[ROLL]);
    double cp = std::cos(state_[PITCH]);
    double sp = std::sin(state_[PITCH]);
    double cy = std::cos(state_[YAW]);
    double sy = std::sin(state_[YAW]);

    // Rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    double nx = (cy * cp) * body_vec.x +
                (cy * sp * sr - sy * cr) * body_vec.y +
                (cy * sp * cr + sy * sr) * body_vec.z;

    double ny = (sy * cp) * body_vec.x +
                (sy * sp * sr + cy * cr) * body_vec.y +
                (sy * sp * cr - cy * sr) * body_vec.z;

    double nz = (-sp) * body_vec.x +
                (cp * sr) * body_vec.y +
                (cp * cr) * body_vec.z;

    return Vector3(nx, ny, nz);
}

double StateEstimator6DOF::normalize_angle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double StateEstimator6DOF::kalman_gain(int state_idx,
                                        double measurement_noise) const {
    // K = P / (P + R)  for scalar case
    double p = covariance_diag_[static_cast<size_t>(state_idx)];
    double r = measurement_noise * measurement_noise;
    return p / (p + r);
}

void StateEstimator6DOF::scalar_update(int state_idx, double innovation,
                                        double measurement_noise) {
    size_t idx = static_cast<size_t>(state_idx);
    double k = kalman_gain(state_idx, measurement_noise);

    // State update: x = x + K * innovation
    state_[idx] += k * innovation;

    // Covariance update: P = (1 - K) * P
    covariance_diag_[idx] *= (1.0 - k);

    // Prevent covariance from going to zero
    covariance_diag_[idx] = std::max(covariance_diag_[idx], 1e-10);
}

}  // namespace submarine_drivers
