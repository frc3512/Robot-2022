// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootFour() {
    // Initial Pose - Right in front of alliance colored ball. The robot starts
    // with one pre-loaded ball.
    frc::Pose2d kInitialPose{8.598_m, 6.081_m,
                             units::radian_t{wpi::numbers::pi / 2}};

    // Intake Pose - Just in front of initial position. Right on top of colored
    // ball.
    frc::Pose2d kFirstIntakePose{8.598_m, 7.247_m,
                                 units::radian_t{wpi::numbers::pi / 2}};

    // Shoot Pose - Right up to the fender of the hub.
    const frc::Pose2d kShootPose{8.459_m, 5.321_m, units::radian_t{-1.917244}};

    // Back Up Pose - Behind and to the left of the shoot pose, redirects the
    // robot.
    const frc::Pose2d kBackUpPose{7.948_m, 6.493_m, units::radian_t{0}};

    // Second Intake Pose - To the right of the starting position. Right on top
    // of second colored ball.
    frc::Pose2d kSecondIntakePose{10.711_m, 6.071_m, units::radian_t{0}};

    // Third Intake Pose - Right in front of the human player station, picks up
    // single ball.
    frc::Pose2d kThirdIntakePose{14.040_m, 6.568_m, units::radian_t{0.406644}};

    drivetrain.Reset(kInitialPose);

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    drivetrain.AddTrajectory(kInitialPose, {}, kFirstIntakePose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();
    intake.Stow();

    drivetrain.SetHeadingGoal(units::radian_t{(3 * wpi::numbers::pi) / 2});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    kFirstIntakePose = UpdateAutoPoseRotation(
        kFirstIntakePose, units::radian_t{(3 * wpi::numbers::pi) / 2});

    drivetrain.AddTrajectory(kFirstIntakePose, {}, kShootPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    Shoot(FrontFlywheelConstants::kShootHighFender,
          BackFlywheelConstants::kShootHighFender);
    SetReadyToShoot(true);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    {
        auto config = DrivetrainController::MakeTrajectoryConfig();
        config.SetReversed(true);
        drivetrain.AddTrajectory(kShootPose, {}, kBackUpPose, config);

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
            return;
        }
    }

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    drivetrain.AddTrajectory(
        {kBackUpPose, kSecondIntakePose, kThirdIntakePose});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();
    intake.Stow();

    drivetrain.SetHeadingGoal(units::radian_t{wpi::numbers::pi});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    kThirdIntakePose = UpdateAutoPoseRotation(
        kThirdIntakePose, units::radian_t{wpi::numbers::pi});
    kSecondIntakePose = UpdateAutoPoseRotation(
        kSecondIntakePose, units::radian_t{wpi::numbers::pi});

    drivetrain.AddTrajectory({kThirdIntakePose, kSecondIntakePose, kShootPose});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    Shoot(FrontFlywheelConstants::kShootHighFender,
          BackFlywheelConstants::kShootHighFender);
    SetReadyToShoot(true);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }
}

}  // namespace frc3512
