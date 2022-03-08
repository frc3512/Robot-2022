// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoDriveForward() {
    frc::Pose2d kInitialPose{2_m, 2_m, units::radian_t{0}};
    frc::Pose2d kSecondPose{5_m, 2_m, units::radian_t{0}};

    drivetrain.Reset(kInitialPose);

    drivetrain.AddTrajectory(kInitialPose, {}, kSecondPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }
}
}  // namespace frc3512
