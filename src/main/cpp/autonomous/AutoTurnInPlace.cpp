// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoTurnInPlace() {
    // Initial Pose - Make is visible on Field2D for testing purposes
    const frc::Pose2d kInitialPose1{10_m, 5.662_m, 0_rad};

    // End Pose - Make is visible on Field2D for testing purposes
    const frc::Pose2d kTurningPose1{12_m, 5.662_m, 0_rad};

    // Initial Pose - Make is visible on Field2D for testing purposes
    const frc::Pose2d kTurningPose2{12_m, 5.662_m,
                                    units::radian_t{wpi::numbers::pi}};

    // End Pose - Make is visible on Field2D for testing purposes
    const frc::Pose2d kEndPose{8_m, 5.662_m, units::radian_t{wpi::numbers::pi}};

    drivetrain.Reset(kInitialPose1);

    drivetrain.AddTrajectory(kInitialPose1, {}, kTurningPose1);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    drivetrain.TurnDrivetrainInPlace(units::radian_t{wpi::numbers::pi});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    drivetrain.AddTrajectory(kTurningPose2, {}, kEndPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    drivetrain.TurnDrivetrainInPlace(units::radian_t{0});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }
}
}  // namespace frc3512
