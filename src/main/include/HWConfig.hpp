// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

/**
 * The hardware configuration namespace is for joystick ports and roboRIO
 * channel assignments.
 *
 * Other hardware configuration constants that won't change often and belong to
 * only one subsystem (like gear ratios) should be defined in the subsystem
 * itself instead.
 */
namespace frc3512::HWConfig {

/// Drive joystick 1 port
constexpr int kDriveStick1Port = 0;

/// Drive joystick 2 port
constexpr int kDriveStick2Port = 1;

/// Appendage joystick 1 port
constexpr int kAppendageStick1Port = 2;

/// Appendage joystick 2 port
constexpr int kAppendageStick2Port = 3;

namespace Drivetrain {
/// Left motor leader CAN ID
constexpr int kLeftMotorLeaderID = 16;

/// Left motor follower CAN ID
constexpr int kLeftMotorFollowerID = 1;

/// Right motor leader CAN ID
constexpr int kRightMotorLeaderID = 14;

/// Right motor follower CAN ID
constexpr int kRightMotorFollowerID = 15;

// Encoder channels
constexpr int kLeftEncoderA = 0;
constexpr int kLeftEncoderB = 1;
constexpr int kRightEncoderA = 2;
constexpr int kRightEncoderB = 3;

/// Left ultrasonic sensor channel
constexpr int kLeftUltrasonicChannel = 3;

/// Right ultrasonic sensor channel
constexpr int kRightUltrasonicChannel = 2;
}  // namespace Drivetrain

namespace Flywheel {
/// Left motor CAN ID
constexpr int kLeftMotorID = 9;

/// Right motor CAN ID
constexpr int kRightMotorID = 10;

/// Encoder channel A
constexpr int kEncoderA = 6;

/// Encoder channel B
constexpr int kEncoderB = 7;
}  // namespace Flywheel

namespace Turret {
/// Turret motor CAN ID
constexpr int kMotorID = 8;

/// Counterclockwise hall sensor digital input channel
constexpr int kCCWHallChannel = 0;

/// Clockwise hall sensor digital input channel
constexpr int kCWHallChannel = 1;

// Encoder digital input channel
constexpr int kEncoderChannel = 8;
}  // namespace Turret

namespace Intake {

/// Lower proximity sensor digital input channel
constexpr int kLowerSensorChannel = 4;

/// Upper proximity sensor digital input channel
constexpr int kUpperSensorChannel = 5;

/// Channal for the Fourbar
constexpr int kFourbarChannel = 2;

/// ID for the conveyor motor
constexpr int kConveyorMotorID = 3;

/// ID for the intake motor
constexpr int kIntakeRollerMotorID = 8;
}  // namespace Intake

namespace Climber {
/// ID for the Left Telescoping Motor
constexpr int kLeftTeleMotorID = 4;

/// ID for the Right Telescoping Motor
constexpr int kRightTeleMotorID = 5;

/// ID for the Left climber Solenoid
constexpr int kLeftTeleSolenoid = 6;

/// ID for the Right climber Solenoid
constexpr int kRightTeleSolenoid = 7;

/// Rev count for CLimber
constexpr int kRevs = 42;

/// ID for CLimber sensor
constexpr int kClimbSensor = 10;
}  // namespace Climber
}  // namespace frc3512::HWConfig
