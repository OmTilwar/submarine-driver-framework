// Copyright (c) 2025 Om Pratap Tilwar
// Licensed under the MIT License. See LICENSE for details.

#pragma once

#include "submarine_drivers/core/actuator_base.hpp"
#include "submarine_drivers/core/types.hpp"

namespace submarine_drivers {

/**
 * @brief Thruster driver with PWM control, thrust curves, and deadband.
 *
 * Models a submarine thruster (e.g., Blue Robotics T200/T500) with:
 *   - PWM input (1100-1900 us) → force output mapping
 *   - Quadratic thrust curve: F = k * |rpm|^2 * sign(rpm)
 *   - Deadband compensation around neutral (1500 us)
 *   - Force saturation at max thrust
 *   - Thermal protection (simple model)
 *
 * The thrust curve maps PWM signal to RPM to force, matching real
 * thruster characteristics where force scales quadratically with RPM.
 */
class ThrusterDriver : public ActuatorBase {
public:
    struct Config {
        std::string thruster_id{"T0"};

        // PWM configuration
        double pwm_neutral{1500.0};    // neutral PWM (us)
        double pwm_min{1100.0};        // minimum PWM (full reverse)
        double pwm_max{1900.0};        // maximum PWM (full forward)
        double deadband_width{25.0};   // deadband around neutral (us)

        // Thrust curve parameters
        double thrust_coefficient{0.01};  // F = k * rpm^2 (N/(rpm^2))
        double max_rpm{3800.0};           // maximum RPM
        double max_thrust{50.0};          // maximum thrust (N)

        // Thermal model
        double max_temperature{85.0};     // thermal cutoff (C)
        double thermal_time_constant{30.0}; // seconds to heat up at full power
        double ambient_temperature{20.0};
    };

    explicit ThrusterDriver(const Config& config = Config{});
    ~ThrusterDriver() override = default;

    // DriverBase interface
    bool initialize() override;
    void update(double dt_seconds) override;
    void shutdown() override;

    // ActuatorBase interface
    void set_safe_state() override;
    bool is_in_safe_state() const override;
    double get_max_output() const override;

    /// Command the thruster with desired force (N)
    ThrusterCommand command_force(double force_newtons);

    /// Command the thruster with raw PWM (microseconds)
    ThrusterCommand command_pwm(double pwm_us);

    /// Get current thruster feedback
    ThrusterFeedback get_feedback() const { return feedback_; }

    /// Convert PWM to force using thrust curve
    double pwm_to_force(double pwm_us) const;

    /// Convert force to PWM
    double force_to_pwm(double force_newtons) const;

    /// Arm/disarm the thruster
    void arm() { armed_ = true; }
    void disarm();

    /// Check if armed
    bool is_armed() const { return armed_; }

private:
    Config config_;
    ThrusterCommand last_command_;
    ThrusterFeedback feedback_;
    double current_temperature_;

    /// Apply deadband compensation
    double apply_deadband(double pwm_us) const;

    /// Update thermal model
    void update_thermal(double dt_seconds, double power_fraction);
};

}  // namespace submarine_drivers
