// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoShootTwo() {
    frc::Pose2d kInitialPose{6.5606_m, 2.711_m, units::radian_t{-135.918_deg}};

    frc::Pose2d kBallPose{5.248_m, 2.076_m, units::radian_t{-138.072_deg}};

    frc::Pose2d kFenderPose{7.572_m, 2.928_m, units::radian_t{72.399_deg}};

    drivetrain.Reset(kInitialPose);

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    drivetrain.AddTrajectory(kInitialPose, {}, kBallPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();
    intake.Stow();

    drivetrain.SetHeadingGoal(units::radian_t{wpi::numbers::pi / 6});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    kBallPose = UpdateAutoPoseRotation(kBallPose,
                                       units::radian_t{wpi::numbers::pi / 6});

    drivetrain.AddTrajectory(kBallPose, {}, kFenderPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    Shoot(FrontFlywheelConstants::kShootHighFender,
          BackFlywheelConstants::kShootHighFender);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }
}
}  // namespace frc3512
