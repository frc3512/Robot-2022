// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/curvature.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/numbers>

/**
 * This namespace is for drivetrain controller constants that
 * only apply to the two controllers themselves. See
 * HWConfig.hpp for hardware configuration.
 */

namespace frc3512::DrivetrainConstants {

/// The wheel radius.
static constexpr units::meter_t kWheelRadius = 3_in;

/// The drivetrain gear ratio from the encoder to the wheel.
static constexpr double kDriveGearRatio = 5.818181 / 1.6;

/// Drivetrain distance per encoder pulse.
static constexpr double kDpP =
    (2.0 * wpi::numbers::pi * kWheelRadius.value()) * kDriveGearRatio / 2048.0;

/// Drivetrain chassis width.
static constexpr units::meter_t kWidth = [] {
    auto absoluteValue = [](auto arg) {
        return arg > decltype(arg){0} ? arg : -1.0 * arg;
    };

    // These values were collected by rotating the robot in place and
    // recording the encoder position and gyro heading measurements.
    // Difference is final measurement minus initial measurement.
    constexpr auto kLeftPosition = 2.18274_m - 0_m;
    constexpr auto kRightPosition = (-1.0 * 2.19665_m) - 0_m;
    constexpr auto kHeading = (-1.0 * 3.1219_rad) - 3.1415_rad;

    return (absoluteValue(kLeftPosition) + absoluteValue(kRightPosition)) /
           absoluteValue(kHeading) * 1_rad;
}();

namespace Meters {

/// Linear velocity system ID gain.
static constexpr auto kLinearV = 1.29407_V / 1_mps;

/// Linear acceleration system ID gain.
static constexpr auto kLinearA = 1.26246_V / 1_mps_sq;

/// Angular velocity system ID gain.
static constexpr auto kAngularV = 2.4142_V / 1_mps;

/// Angular acceleration system ID gain.
static constexpr auto kAngularA = 1.52403_V / 1_mps_sq;

/// Maximum linear velocity.
static constexpr auto kMaxLinearV = 12_V / kLinearV;

/// Maximum linear acceleration.
static constexpr auto kMaxLinearA = 12_V / kLinearA;

}  // namespace Meters

namespace Radians {

/// Angular velocity system ID gain.
static constexpr auto kAngularV = 2.4142_V / 1_rad_per_s;

/// Angular acceleration system ID gain.
static constexpr auto kAngularA = 1.52403_V / 1_rad_per_s_sq;

/// Maximum angular velocity.
static constexpr auto kMaxAngularV = 12_V / kAngularV;

/// Maximum angular acceleration.
static constexpr auto kMaxAngularA = 12_V / kAngularA;

}  // namespace Radians

}  // namespace frc3512::DrivetrainConstants
