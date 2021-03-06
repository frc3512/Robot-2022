// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoShootOne() {
    /// Initial Pose - against the hub. The robots starts pre-loaded with one
    /// ball.
    frc::Pose2d kInitialPose{2.5_m, 2_m, 0_rad};
    /// Backup Pose - drive back off the tarmac
    frc::Pose2d kBackupPose{1.0_m, 2_m, 0_rad};

    drivetrain.Reset(kInitialPose);

    auto config = Drivetrain::MakeTrajectoryConfig();
    config.SetReversed(true);

    drivetrain.AddTrajectory(kInitialPose, {}, kBackupPose, config);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    // shoots one preloaded ball.
    Shoot(FrontFlywheelConstants::kShootHighFender,
          BackFlywheelConstants::kShootHighFender, true);
    // shoots as soon as flywheels are up to speed.
    SetReadyToShoot(true);
}
}  // namespace frc3512
