#include "Robot.hpp"

namespace frc3512
{
    void Robot::AutoTurnInPlace()
    {
        frc::Pose2d kInitialPose{2_m, 3_m, 0_rad};

        drivetrain.Reset(kInitialPose);

        drivetrain.SetHeadingGoal(units::radian_t{wpi::numbers::pi});

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); }))
        {
            return; 
        }
    }
}