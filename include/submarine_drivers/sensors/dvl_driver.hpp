// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/noise_model.hpp"
#include "submarine_drivers/core/sensor_base.hpp"
#include "submarine_drivers/core/types.hpp"
#include <memory>

namespace submarine_drivers {

/**
 * @brief DVL (Doppler Velocity Log) driver stub.
 *
 * Simulates a DVL providing bottom-track velocity measurements in the
 * body frame and altitude above the seafloor. Real DVLs (e.g., Teledyne
 * RDI Explorer, Nortek DVL1000) use acoustic beams to measure velocity
 * relative to the bottom.
 *
 * The stub generates readings from ground-truth velocity with configurable
 * noise and simulates bottom-lock loss at configurable altitudes.
 */
class DvlDriver : public SensorBase {
public:
    struct Config {
        double update_rate_hz{5.0};           // typical DVL rate (lower than IMU)
        double max_altitude{200.0};           // max altitude for bottom lock (m)
        double min_altitude{0.3};             // min altitude for valid readings (m)

        // Velocity noise (per-axis)
        NoiseModel::Config velocity_noise;

        // Altitude noise
        NoiseModel::Config altitude_noise;
    };

    DvlDriver() : DvlDriver(Config{}) {}
    explicit DvlDriver(const Config& config);
    ~DvlDriver() override = default;

    // DriverBase interface
    bool initialize() override;
    void update(double dt_seconds) override;
    void shutdown() override;

    // SensorBase interface
    std::chrono::steady_clock::time_point get_last_timestamp() const override;
    bool has_valid_reading() const override;

    /// Get the latest DVL reading
    DvlReading get_reading() const { return last_reading_; }

    /// Set ground-truth velocity and altitude (for simulation)
    void set_ground_truth(const Vector3& velocity, double altitude);

    /// Check if bottom lock is currently available
    bool has_bottom_lock() const { return last_reading_.bottom_lock; }

private:
    Config config_;
    DvlReading last_reading_;
    Vector3 true_velocity_;
    double true_altitude_{50.0};

    std::unique_ptr<NoiseModel> vel_noise_x_;
    std::unique_ptr<NoiseModel> vel_noise_y_;
    std::unique_ptr<NoiseModel> vel_noise_z_;
    std::unique_ptr<NoiseModel> alt_noise_;
};

}  // namespace submarine_drivers
