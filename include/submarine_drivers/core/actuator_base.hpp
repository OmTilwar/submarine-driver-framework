// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/driver_base.hpp"
#include "submarine_drivers/core/types.hpp"

namespace submarine_drivers {

/**
 * @brief Abstract interface for actuator drivers.
 *
 * Extends DriverBase with actuator-specific methods: commanding output,
 * reading feedback, and emergency stop.
 */
class ActuatorBase : public DriverBase {
public:
    explicit ActuatorBase(const std::string& name) : DriverBase(name) {}
    ~ActuatorBase() override = default;

    /// Set the actuator to a safe/neutral state
    virtual void set_safe_state() = 0;

    /// Check if the actuator is in a safe/neutral state
    virtual bool is_in_safe_state() const = 0;

    /// Get the maximum output the actuator can produce
    virtual double get_max_output() const = 0;

protected:
    bool armed_{false};
};

}  // namespace submarine_drivers
