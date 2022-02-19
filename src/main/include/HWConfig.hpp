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
constexpr int kLeftMotorLeaderID = 14;

/// Left motor follower CAN ID
constexpr int kLeftMotorFollowerID = 15;

/// Right motor leader CAN ID
constexpr int kRightMotorLeaderID = 16;

/// Right motor follower CAN ID
constexpr int kRightMotorFollowerID = 1;

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
/// Front motor CAN ID
constexpr int kFrontMotorID = 10;

/// Back motor CAN ID
constexpr int kBackMotorID = 5;

/// Encoder channel A
constexpr int kEncoderA = 6;

/// Encoder channel B
constexpr int kEncoderB = 7;
}  // namespace Flywheel

namespace Intake {
/// Arm motor CAN ID
constexpr int kArmMotorID = 9;

/// Right Conveyor motor CAN ID
constexpr int kConveyorRightMotorID = 4;

/// Left Conveyor motor CAN ID
constexpr int kConveyorLeftMotorID = 11;

/// Lower proximity sensor digital input channel
constexpr int kLowerSensorChannel = 4;

/// Upper proximity sensor digital input channel
constexpr int kUpperSensorChannel = 5;

/// Channal for the Fourbar
constexpr int kFourbarChannel = 5;

/// ID for the left conveyor motor
constexpr int kLeftConveyorMotorID = 11;

// ID for the right conveyor motor
constexpr int kRightConveyorMotorID = 4;

}  // namespace Intake

namespace Climber {
/// Right Climber CAN ID
constexpr int kRightClimbMotorID = 2;

constexpr int kLeftClimbMotorID = 13;
}  // namespace Climber

}  // namespace frc3512::HWConfig
