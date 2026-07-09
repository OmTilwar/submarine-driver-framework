// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

namespace submarine_drivers {

/// 3D vector for positions, velocities, forces
struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vector3() = default;
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};

/// Euler angles (roll, pitch, yaw) in radians
struct EulerAngles {
    double roll{0.0};   // rotation about x-axis
    double pitch{0.0};  // rotation about y-axis
    double yaw{0.0};    // rotation about z-axis

    EulerAngles() = default;
    EulerAngles(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
};

/// Full 6-DOF pose: position (x, y, z) + orientation (roll, pitch, yaw)
struct Pose6DOF {
    Vector3 position;
    EulerAngles orientation;
    std::chrono::steady_clock::time_point timestamp;

    Pose6DOF() : timestamp(std::chrono::steady_clock::now()) {}
};

/// 6-DOF velocity: linear (u, v, w) + angular (p, q, r)
struct Velocity6DOF {
    Vector3 linear;   // body-frame: surge (u), sway (v), heave (w)
    Vector3 angular;  // body-frame: roll rate (p), pitch rate (q), yaw rate (r)
};

/// IMU measurement: accelerometer + gyroscope
struct ImuReading {
    Vector3 acceleration;     // m/s^2, body frame
    Vector3 angular_velocity; // rad/s, body frame
    std::chrono::steady_clock::time_point timestamp;
    bool valid{true};
};

/// DVL measurement: bottom-track velocity + altitude
struct DvlReading {
    Vector3 velocity;    // m/s, body frame (bottom-track)
    double altitude;     // meters above bottom
    bool bottom_lock;    // true if bottom-tracking is valid
    std::chrono::steady_clock::time_point timestamp;
    bool valid{true};
};

/// Depth sensor measurement: pressure-derived depth + temperature
struct DepthReading {
    double depth;        // meters below surface (positive down)
    double temperature;  // degrees Celsius (for compensation)
    double pressure;     // raw pressure in Pascals
    std::chrono::steady_clock::time_point timestamp;
    bool valid{true};
};

/// Thruster command
struct ThrusterCommand {
    double force_newtons{0.0};  // desired force output (N)
    double pwm_microseconds{1500.0}; // PWM signal (1100-1900 us typical)
};

/// Thruster feedback
struct ThrusterFeedback {
    double actual_force{0.0};     // estimated actual force (N)
    double current_draw{0.0};     // amperes
    double temperature{25.0};     // motor temperature (C)
    bool healthy{true};
};

/// Driver health status
enum class DriverStatus : uint8_t {
    UNINITIALIZED = 0,
    INITIALIZING,
    RUNNING,
    WARNING,
    ERROR,
    SHUTDOWN
};

/// Convert DriverStatus to string for logging
inline std::string status_to_string(DriverStatus status) {
    switch (status) {
        case DriverStatus::UNINITIALIZED: return "UNINITIALIZED";
        case DriverStatus::INITIALIZING:  return "INITIALIZING";
        case DriverStatus::RUNNING:       return "RUNNING";
        case DriverStatus::WARNING:       return "WARNING";
        case DriverStatus::ERROR:         return "ERROR";
        case DriverStatus::SHUTDOWN:      return "SHUTDOWN";
        default: return "UNKNOWN";
    }
}

}  // namespace submarine_drivers
