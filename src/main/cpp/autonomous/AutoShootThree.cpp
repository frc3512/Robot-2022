// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootThree() {
    // Initial Pose - Right in front of alliance colored ball. The robot starts
    // pre-loaded with one ball.
    const frc::Pose2d kInitialPose{8.598_m, 6.081_m,
                                   units::radian_t{wpi::numbers::pi / 2}};

    // Intake Pose - Just in front of the initial position. Right on top of the
    // colored ball.
    frc::Pose2d kIntakePose{8.598_m, 7.231_m,
                            units::radian_t{wpi::numbers::pi / 2}};

    // Shoot Pose - Right on top of the third ball.
    const frc::Pose2d kEndPose{11.068_m, 6.177_m, units::radian_t{0}};

    drivetrain.Reset(kInitialPose);

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    autonTimer.Start();

    if (!m_autonChooser.Suspend([=] { return autonTimer.HasElapsed(0.5_s); })) {
        return;
    }

    autonTimer.Reset();
    autonTimer.Stop();

    drivetrain.AddTrajectory(kInitialPose, {}, kIntakePose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    autonTimer.Start();
    if (!m_autonChooser.Suspend([=] { return autonTimer.HasElapsed(1.0_s); })) {
        return;
    }

    autonTimer.Reset();
    autonTimer.Stop();

    intake.Stop();
    intake.Stow();

    drivetrain.SetTurningTolerance(0.25_rad);
    drivetrain.SetHeadingGoal(units::radian_t{(3 * wpi::numbers::pi) / 2});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    drivetrain.SetTurningTolerance(0.15_rad);

    Shoot(FrontFlywheelConstants::kShootHighTarmac,
          FrontFlywheelConstants::kShootHighTarmac, true);
    SetReadyToShoot(true);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    kIntakePose = UpdateAutoPoseRotation(kIntakePose, drivetrain.GetAngle());

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    autonTimer.Start();
    if (!m_autonChooser.Suspend([=] { return autonTimer.HasElapsed(0.5_s); })) {
        return;
    }

    autonTimer.Reset();
    autonTimer.Stop();

    drivetrain.AddTrajectory(kIntakePose, {}, kEndPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    autonTimer.Start();
    if (!m_autonChooser.Suspend([=] { return autonTimer.HasElapsed(0.5_s); })) {
        return;
    }

    autonTimer.Reset();
    autonTimer.Stop();

    intake.Stow();
    intake.Stop();

    drivetrain.SetTurningTolerance(0.25_rad);
    drivetrain.SetHeadingGoal(units::radian_t{(2.5 * wpi::numbers::pi) / 2.0});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    drivetrain.SetTurningTolerance(0.15_rad);
    Shoot(FrontFlywheelConstants::kShootHighTarmac,
          BackFlywheelConstants::kShootHighTarmac, true);
    SetReadyToShoot(true);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }
}

}  // namespace frc3512
