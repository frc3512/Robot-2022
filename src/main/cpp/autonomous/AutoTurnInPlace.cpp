// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoTurnInPlace() {
    frc::Pose2d kInitialPose{2_m, 2_m, units::radian_t{0}};

    drivetrain.Reset(kInitialPose);

    drivetrain.SetHeadingGoal(units::radian_t{wpi::numbers::pi});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }
}
}  // namespace frc3512
