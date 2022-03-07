#include "Robot.hpp"

namespace frc3512
{
    void Robot::AutoDriveForward()
    {
        frc::Pose2d kInitialPose(2_m, 3_m, units::radian_t{0});

        frc::Pose2d kSecondPose(4_m, 3_m, units::radian_t{0});

        drivetrain.Reset(kInitialPose);

        drivetrain.AddTrajectory(kInitialPose, {}, kSecondPose);

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); }))
        {
            return;
        }
    }
}