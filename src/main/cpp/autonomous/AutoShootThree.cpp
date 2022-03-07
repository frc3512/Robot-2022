// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootThree() {
    // Initial Pose - Right up against the Hub
    const frc::Pose2d kInitialPose{8.757_m, 5.319_m,
                                   units::radian_t{83 * wpi::numbers::pi / 60}};

    // Intake Pose - right in front of the ball to turn to.
    frc::Pose2d kIntakePose{8.95_m, 7.2_m,
                            units::radian_t{3 * wpi::numbers::pi / 2}};

    // Intake Pose 2 - intake the ball in front.
    frc::Pose2d kIntake2Pose{8.95_m, 7.5_m,
                             units::radian_t{wpi::numbers::pi / 2}};

    // Intake Pose 3 - intake the ball in front.
    frc::Pose2d kIntake3Pose{11.47_m, 6.30_m,
                             units::radian_t{2 * wpi::numbers::pi / 15}};

    // End Pose - Within the Tarmac
    const frc::Pose2d kEndPose{9.12_m, 5.54_m,
                               units::radian_t{wpi::numbers::pi / 4}};

    drivetrain.Reset(kInitialPose);

    auto config = DrivetrainController::MakeTrajectoryConfig();
    config.SetReversed(true);

    drivetrain.AddTrajectory(kInitialPose, {}, kIntakePose, config);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    drivetrain.SetHeadingGoal(units::radian_t{wpi::numbers::pi / 2});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    kIntakePose = {8.95_m, 7.2_m, units::radian_t{wpi::numbers::pi / 2}};

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    drivetrain.AddTrajectory(kIntakePose, {}, kIntake2Pose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();
    intake.Stow();

    drivetrain.AddTrajectory(kIntake2Pose, {}, kEndPose, config);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    drivetrain.SetHeadingGoal(units::radian_t{83 * wpi::numbers::pi / 60});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    /*
    Shoot(true);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    SetReadyToShoot(true);
    */

    drivetrain.SetHeadingGoal(units::radian_t{2 * wpi::numbers::pi / 15});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }

    intake.Deploy();
    intake.Start(Intake::IntakeDirection::kIntake);

    drivetrain.AddTrajectory(kEndPose, {}, kIntake3Pose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();
    intake.Stow();

    drivetrain.AddTrajectory(kIntake3Pose, {}, kEndPose, config);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    drivetrain.SetHeadingGoal(units::radian_t{83 * wpi::numbers::pi / 60});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }
}

}  // namespace frc3512
