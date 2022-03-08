// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootOne() {
    // Initial Pose - Right in line with the Loading Zone
    const frc::Pose2d kInitialPose{8.44_m, 5.25_m, 0_rad};
    // End Pose - Drive forward slightly
    const frc::Pose2d kEndPose{8.44_m - 2.5 * Drivetrain::kLength, 5.25_m,
                               0_rad};

    drivetrain.Reset(kInitialPose);

    Shoot(FrontFlywheelConstants::kShootHighFender, BackFlywheelConstants::kShootHighFender);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    SetReadyToShoot(true);

    auto config = DrivetrainController::MakeTrajectoryConfig();
    config.SetReversed(true);

    drivetrain.AddTrajectory(kInitialPose, {}, kEndPose, config);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }
}

}  // namespace frc3512