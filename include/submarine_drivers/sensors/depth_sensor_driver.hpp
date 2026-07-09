// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/noise_model.hpp"
#include "submarine_drivers/core/sensor_base.hpp"
#include "submarine_drivers/core/types.hpp"
#include <memory>

namespace submarine_drivers {

/**
 * @brief Depth sensor driver (pressure-based).
 *
 * Simulates a pressure-based depth sensor that converts raw pressure
 * readings to depth below the surface. Includes temperature compensation
 * for water density variations (seawater density changes with temperature
 * and salinity).
 *
 * Models: pressure_raw → temperature compensation → depth_meters
 *
 * Based on the hydrostatic equation: depth = (P - P_atm) / (rho * g)
 * where rho varies with temperature and salinity.
 */
class DepthSensorDriver : public SensorBase {
public:
    struct Config {
        double update_rate_hz{10.0};
        double atmospheric_pressure{101325.0};  // Pa (sea level)
        double water_density{1025.0};           // kg/m^3 (seawater)
        double gravity{9.80665};                // m/s^2

        // Temperature compensation coefficients
        double density_temp_coeff{-0.2};  // kg/m^3 per degree C deviation from 15C
        double reference_temp{15.0};      // reference temperature (C)

        // Noise models
        NoiseModel::Config pressure_noise;
        NoiseModel::Config temperature_noise;
    };

    explicit DepthSensorDriver(const Config& config = Config{});
    ~DepthSensorDriver() override = default;

    // DriverBase interface
    bool initialize() override;
    void update(double dt_seconds) override;
    void shutdown() override;

    // SensorBase interface
    std::chrono::steady_clock::time_point get_last_timestamp() const override;
    bool has_valid_reading() const override;

    /// Get the latest depth reading
    DepthReading get_reading() const { return last_reading_; }

    /// Set ground-truth depth and temperature (for simulation)
    void set_ground_truth(double depth_meters, double temperature_celsius);

    /// Convert raw pressure to depth with temperature compensation
    double pressure_to_depth(double pressure_pa, double temperature_c) const;

private:
    Config config_;
    DepthReading last_reading_;
    double true_depth_{0.0};
    double true_temperature_{15.0};

    std::unique_ptr<NoiseModel> pressure_noise_;
    std::unique_ptr<NoiseModel> temperature_noise_;

    /// Calculate water density with temperature compensation
    double compensated_density(double temperature_c) const;
};

}  // namespace submarine_drivers
