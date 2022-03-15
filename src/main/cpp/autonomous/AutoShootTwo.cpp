// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootTwo() {
    // Initial Pose - Right in line with the Loading Zone
    const frc::Pose2d kInitialPose{8.598_m, 6.081_m,
                                   units::radian_t{wpi::numbers::pi / 2}};

    // Intake Pose - right in front of the ball to turn to.
    frc::Pose2d kIntakePose{8.598_m, 7.247_m,
                            units::radian_t{wpi::numbers::pi / 2}};

    // Initial Pose - Right in line with the Loading Zone
    const frc::Pose2d kEndPose{8.459_m, 5.321_m,
                               units::radian_t{-1.917244}};

    drivetrain.Reset(kInitialPose);

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    drivetrain.AddTrajectory(kInitialPose, {}, kIntakePose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); }))
    {
        return;
    }

    intake.Stop();
    intake.Stow();

    drivetrain.SetHeadingGoal(units::radian_t{(3 * wpi::numbers::pi) / 2});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); }))
    {
        return;
    }

    kIntakePose = UpdateAutoPoseRotation(kIntakePose, units::radian_t{(3 * wpi::numbers::pi) / 2});

    drivetrain.AddTrajectory(kIntakePose, {}, kEndPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); }))
    {
        return; 
    }

    Shoot(FrontFlywheelConstants::kShootHighFender, BackFlywheelConstants::kShootHighFender);
    SetReadyToShoot(true);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); }))
    {
        return;
    }
}

}  // namespace frc3512
