// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootFour() {
    // Initial Pose - Right in front of alliance colored ball. The robot starts
    // pre-loaded with one ball.
    const frc::Pose2d kInitialPose{8.598_m, 6.081_m,
                                   units::radian_t{wpi::numbers::pi / 2}};

    // Intake Pose - Just in front of the initial position. Right on top of the
    // colored ball.
    frc::Pose2d kIntakePose{8.598_m, 7.231_m,
                            units::radian_t{wpi::numbers::pi / 2}};

    // Third Ball - Right on top of the third ball.
    frc::Pose2d kThirdBall{11.068_m, 6.518_m, units::radian_t{-5.524}};

    /// Fourth Ball - Right in front of the human player station on top of the
    /// ball.
    const frc::Pose2d kFourthBall{14.040_m, 6.568_m, units::radian_t{0.406644}};

    // End Pose - The last position, between the third and fourth ball. Farther
    // than the third ball from the goal so the robot doesn't have to travel as
    // far.
    const frc::Pose2d kEndPose{12.568_m, 6.177_m,
                               units::radian_t{(3.0 * wpi::numbers::pi) / 2.0}};

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
    if (!m_autonChooser.Suspend([=] { return autonTimer.HasElapsed(0.5_s); })) {
        return;
    }

    autonTimer.Reset();
    autonTimer.Stop();

    intake.Stop();
    intake.Stow();

    drivetrain.SetHeadingGoal(units::radian_t{(3 * wpi::numbers::pi) / 2});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

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

    drivetrain.AddTrajectory({kIntakePose, kThirdBall, kFourthBall});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    autonTimer.Start();
    if (!m_autonChooser.Suspend([=] { return autonTimer.HasElapsed(0.5_s); })) {
        return;
    }

    autonTimer.Reset();
    autonTimer.Stop();
    intake.Stop();
    intake.Stow();

    {
        auto config = DrivetrainController::MakeTrajectoryConfig();
        config.SetReversed(true);

        drivetrain.AddTrajectory(kFourthBall, {}, kEndPose, config);

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
            return;
        }
    }

    Shoot(FrontFlywheelConstants::kShootHighTarmac,
          BackFlywheelConstants::kShootHighTarmac, true);
    SetReadyToShoot(true);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }
}

}  // namespace frc3512
