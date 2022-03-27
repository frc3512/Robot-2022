// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

namespace frc3512 {
/**
 * Flywheel physical robot placement.
 */
enum class FlywheelPose { kFront, kBack };

namespace FrontFlywheelConstants {
/// Static friction system ID gain.
static constexpr auto kS = 0.18296_V;

/// Angular velocity system ID gain.
static constexpr auto kV = 0.020245_V / 1_rad_per_s;

/// Angular acceleration system ID gain.
static constexpr auto kA = 0.003333_V / 1_rad_per_s_sq;

/// Maximum front flywheel angular velocity.
static constexpr auto kMaxAngularVelocity = 12_V / kV;

/// High goal speed for front shooter when doing fender shot.
static constexpr units::radians_per_second_t kShootHighFender = 359_rad_per_s;

/// High goal speed for front shooter when farther from target.
static constexpr units::radians_per_second_t kShootHighTarmac = 359_rad_per_s;

/// Low goal sped for front shooter.
static constexpr units::radians_per_second_t kShootLow = 240_rad_per_s;
}  // namespace FrontFlywheelConstants

namespace BackFlywheelConstants {
/// Static friction system ID gain.
static constexpr auto kS = 0.070473_V;

/// Angular velocity system ID gain.
static constexpr auto kV = 0.021752_V / 1_rad_per_s;

/// Angular acceleration system ID gain.
static constexpr auto kA = 0.0095716_V / 1_rad_per_s_sq;

/// Maximum back flywheel angular velocity.
static constexpr auto kMaxAngularVelocity = 12_V / kV;

/// High goal speed for back shooter when doing fender shot.
static constexpr units::radians_per_second_t kShootHighFender = 100_rad_per_s;

/// High goal speed for back shooter when farther from target.
static constexpr units::radians_per_second_t kShootHighTarmac = 240_rad_per_s;

/// Low goal speed for back shooter.
static constexpr units::radians_per_second_t kShootLow = 290_rad_per_s;
}  // namespace BackFlywheelConstants
}  // namespace frc3512
