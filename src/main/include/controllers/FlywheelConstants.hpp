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
static constexpr auto kS = 0.11466_V;

/// Angular velocity system ID gain.
static constexpr auto kV = 0.013245_V / 1_rad_per_s;

/// Angular acceleration system ID gain.
static constexpr auto kA = 0.00067254_V / 1_rad_per_s_sq;

/// Maximum front flywheel angular velocity.
static constexpr auto kMaxAngularVelocity = 12_V / kV;

/// High goal speed for front shooter.
static constexpr units::radians_per_second_t kShootHigh = 285_rad_per_s;

/// Low goal sped for front shooter.
static constexpr units::radians_per_second_t kShootLow = 125_rad_per_s;
}  // namespace FrontFlywheelConstants

namespace BackFlywheelConstants {
/// Static friction system ID gain.
static constexpr auto kS = 0.0052082_V;

/// Angular velocity system ID gain.
static constexpr auto kV = 0.013888_V / 1_rad_per_s;

/// Angular acceleration system ID gain.
static constexpr auto kA = 0.00085486_V / 1_rad_per_s_sq;

/// Maximum back flywheel angular velocity.
static constexpr auto kMaxAngularVelocity = 12_V / kV;

/// High goal speed for back shooter.
static constexpr units::radians_per_second_t kShootHigh = 630_rad_per_s;

/// Low goal speed for back shooter.
static constexpr units::radians_per_second_t kShootLow = 125_rad_per_s;
}  // namespace BackFlywheelConstants
}  // namespace frc3512
