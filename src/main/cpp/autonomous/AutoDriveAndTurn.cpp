#include "Robot.hpp"

namespace frc3512
{
    void Robot::AutoDriveAndTurn()
    {
        frc::Pose2d kInitialPose{2_m, 2_m, 0_rad};
        frc::Pose2d kSecondPose{5_m, 4_m, 0_rad};

        drivetrain.Reset(kInitialPose);
        drivetrain.AddTrajectory(kInitialPose, {}, kSecondPose);

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); }))
        {
            return;
        }

        kInitialPose = UpdateAutoPoseRotation(kInitialPose, units::radian_t{wpi::numbers::pi});
        kSecondPose = UpdateAutoPoseRotation(kSecondPose, units::radian_t{wpi::numbers::pi});

        drivetrain.SetHeadingGoal(units::radian_t{wpi::numbers::pi});

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); }))
        {
            return;
        }

        drivetrain.AddTrajectory(kSecondPose, {}, kInitialPose);

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); }))
        {
            return;
        }
    }
}