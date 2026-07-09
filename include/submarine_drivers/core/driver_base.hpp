// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/types.hpp"
#include <memory>
#include <string>

namespace submarine_drivers {

/**
 * @brief Abstract base class for all hardware drivers.
 *
 * Defines the lifecycle interface (initialize → update → shutdown) that all
 * sensor and actuator drivers must implement. Uses RAII principles — resources
 * acquired in initialize() are released in shutdown().
 */
class DriverBase {
public:
    explicit DriverBase(const std::string& name) : name_(name) {}
    virtual ~DriverBase() = default;

    // Non-copyable, movable
    DriverBase(const DriverBase&) = delete;
    DriverBase& operator=(const DriverBase&) = delete;
    DriverBase(DriverBase&&) = default;
    DriverBase& operator=(DriverBase&&) = default;

    /// Initialize the driver (acquire resources, configure hardware)
    virtual bool initialize() = 0;

    /// Update the driver state (called at the driver's update rate)
    virtual void update(double dt_seconds) = 0;

    /// Shutdown the driver (release resources, safe state)
    virtual void shutdown() = 0;

    /// Get driver name
    const std::string& get_name() const { return name_; }

    /// Get current driver status
    DriverStatus get_status() const { return status_; }

    /// Check if driver is healthy and running
    bool is_healthy() const {
        return status_ == DriverStatus::RUNNING ||
               status_ == DriverStatus::WARNING;
    }

protected:
    std::string name_;
    DriverStatus status_{DriverStatus::UNINITIALIZED};
};

}  // namespace submarine_drivers
