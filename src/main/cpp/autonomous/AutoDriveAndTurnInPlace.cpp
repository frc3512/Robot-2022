#include "Robot.hpp"

namespace frc3512
{
    void Robot::AutoDriveAndTurnInPlace()
    {
        frc::Pose2d kInitialPose{2_m, 3_m, 0_rad};

        frc::Pose2d kSecondPose{5_m, 3_m, 0_rad};

        drivetrain.Reset(kInitialPose);

        drivetrain.AddTrajectory({kInitialPose, kSecondPose});

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); }))
        {
            return;
        }

        drivetrain.SetHeadingGoal(units::radian_t{wpi::numbers::pi});

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); }))
        {
            return;
        }

        kInitialPose = UpdatePoseRotation(kInitialPose, units::radian_t{wpi::numbers::pi});
        kSecondPose = UpdatePoseRotation(kSecondPose, units::radian_t{wpi::numbers::pi});

        drivetrain.AddTrajectory({kSecondPose, kInitialPose});

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); }))
        {
            return;
        }
    }
}