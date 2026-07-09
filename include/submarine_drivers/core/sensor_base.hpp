// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/driver_base.hpp"
#include "submarine_drivers/core/types.hpp"
#include <chrono>

namespace submarine_drivers {

/**
 * @brief Abstract interface for all sensor drivers.
 *
 * Extends DriverBase with sensor-specific methods: reading data,
 * checking timestamps, and querying measurement validity.
 */
class SensorBase : public DriverBase {
public:
    explicit SensorBase(const std::string& name, double update_rate_hz = 100.0)
        : DriverBase(name), update_rate_hz_(update_rate_hz) {}

    ~SensorBase() override = default;

    /// Get the sensor's configured update rate (Hz)
    double get_update_rate() const { return update_rate_hz_; }

    /// Get timestamp of the last valid reading
    virtual std::chrono::steady_clock::time_point get_last_timestamp() const = 0;

    /// Check if the last reading is valid
    virtual bool has_valid_reading() const = 0;

    /// Get time since last valid reading (seconds)
    double get_age_seconds() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(now - get_last_timestamp());
        return duration.count();
    }

    /// Check if data is stale (older than timeout)
    bool is_stale(double timeout_seconds = 1.0) const {
        return get_age_seconds() > timeout_seconds;
    }

protected:
    double update_rate_hz_;
};

}  // namespace submarine_drivers
