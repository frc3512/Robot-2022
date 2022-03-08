// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoCurveDrive() {
    frc::Pose2d kInitialPose{2_m, 2_m, units::radian_t{0}};

    frc::Pose2d kSecondPose{7_m, 4_m, units::radian_t{0}};

    drivetrain.Reset(kInitialPose);

    drivetrain.AddTrajectory(kInitialPose, {}, kSecondPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    fmt::print("Robot Pose X: {}, Y: {}, Heading: {}",
               drivetrain.GetPose().X().value(),
               drivetrain.GetPose().Y().value(),
               drivetrain.GetPose().Rotation().Radians().value());
}
}  // namespace frc3512
