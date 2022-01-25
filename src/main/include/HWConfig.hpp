// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {

namespace HWConfig {

/// ID for the intake motor
constexpr int kIntakeMotorID = 0;

/// Channal for the Fourbar
constexpr int kFourbarChannel = 1;

/// ID for the conveyor motor
constexpr int kConveyorMotorID = 2;

/// ID for the Left Telescoping Motor
constexpr int kLeftTeleMotorID = 3;

/// ID for the Right Telescoping Motor
constexpr int kRightTeleMotorID = 4;

/// ID for the Left climber Solenoid
constexpr int kLeftTeleSolenoid = 5;

/// ID for the Right climber Solenoid
constexpr int kRightTeleSolenoid = 6;

/// Temp value for some sort of sensor to determine wheather or not o keep
/// extending
constexpr bool kTEMPSensorVal = true;

}  // namespace HWConfig
}  // namespace frc3512
