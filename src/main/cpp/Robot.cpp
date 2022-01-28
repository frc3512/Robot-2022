// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <stdexcept>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "RealTimePriorities.hpp"
#include "logging/CSVUtil.hpp"

namespace frc3512 {

Robot::Robot() {}

Robot::~Robot() {}

frc::DigitalInput upperSensor;
frc::DigitalInput lowerSensor;
frc::DigitalInput barSensor;

units::second_t Robot::SelectedAutonomousDuration() const {
    return m_autonChooser.SelectedAutonomousDuration();
}

void Robot::SimulationInit() { SubsystemBase::RunAllSimulationInit(); }

void Robot::DisabledInit() {
    m_autonChooser.ResumeAutonomous();
    SubsystemBase::RunAllDisabledInit();
}

void Robot::AutonomousInit() {
    SubsystemBase::RunAllAutonomousInit();
    m_autonChooser.AwaitAutonomous();
}

void Robot::TeleopInit() {
    m_autonChooser.CancelAutonomous();
    SubsystemBase::RunAllTeleopInit();
}

void Robot::TestInit() {
    m_autonChooser.ResumeAutonomous();
    SubsystemBase::RunAllTestInit();
}

void Robot::RobotPeriodic() { SubsystemBase::RunAllRobotPeriodic(); }

void Robot::SimulationPeriodic() { SubsystemBase::RunAllSimulationPeriodic(); }

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonChooser.ResumeAutonomous();
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllTeleopPeriodic();

    static frc::Joystick appendageStick1{HWConfig::appendageStick1PortID};
    static frc::Joystick driveStick1{HWConfig::driveStick1PortID};

    if (appendageStick1.GetRawButton(1)) {
        m_state = ClimbingStates::kSecondRung;
    }

    ClimbingSequenceSM();
}

void Robot::TestPeriodic() { SubsystemBase::RunAllTestPeriodic(); }

void Robot::SelectAutonomous(std::string_view name) {
    m_autonChooser.SelectAutonomous(name);
}

const std::vector<std::string>& Robot::GetAutonomousNames() const {
    return m_autonChooser.GetAutonomousNames();
}

void Robot::ExpectAutonomousEndConds() {
    if constexpr (IsSimulation()) {
        EXPECT_FALSE(m_autonChooser.IsSuspended())
            << "Autonomous mode didn't finish within the autonomous period";

        // EXPECT_TRUE(drivetrain.AtGoal());

        // Verify left/right wheel velocities are close to zero
        /* EXPECT_NEAR(
            drivetrain.GetStates()(DrivetrainController::State::kLeftVelocity),
            0.0, 0.01); */
        /* EXPECT_NEAR(
            drivetrain.GetStates()(DrivetrainController::State::kRightVelocity),
            0.0, 0.01); */
    }
}

void Robot::ClimbingSequenceSM() {
    switch (m_state) {
        case ClimbingStates::kSecondRung: {
            solenoidTimer.Start();
            if (solenoidTimer.HasElapsed(1_s)) {
                climber.TelescopingOut();
<<<<<<< Updated upstream
            } else if ((!barSensor.Get()) && (!upperSensor.Get())) {
                climber.TelescopingExtention(0.65);
            }

            if ((barSensor.Get()) && (!lowerSensor.Get())) {
                climber.TelescopingExtention(-0.65);
            }

            if (lowerSensor.Get()) {
=======
                solenoidTimer.Stop();
                solenoidTimer.Reset();
            } else if ((!climber.BarSensor()) && (!climber.UppperSensor())) {
                climber.TelescopingExtention(0.65);
            }

            if ((climber.BarSensor()) && (!climber.LowerSensor())) {
                climber.TelescopingExtention(-0.65);
            }

            if (climber.LowerSensor()) {
                climber.TelescopingExtention(0.00);
>>>>>>> Stashed changes
                climber.TelescopingIn();
                m_state = ClimbingStates::kThridRung;
            }
            break;
        }
        case ClimbingStates::kThridRung: {
<<<<<<< Updated upstream
=======
            solenoidTimer.Start();
            if (solenoidTimer.HasElapsed(1_s)) {
                climber.TelescopingOut();
                solenoidTimer.Stop();
                solenoidTimer.Reset();
            } else if ((!climber.BarSensor()) && (!climber.UppperSensor())) {
                climber.TelescopingExtention(0.65);
            }

            if ((climber.BarSensor()) && (!climber.LowerSensor())) {
                climber.TelescopingExtention(-0.65);
            }

            if (climber.LowerSensor()) {
                climber.TelescopingExtention(0.00);
                climber.TelescopingIn();
                m_state = ClimbingStates::kTraversalRung;
            }
            break;
        }
        case ClimbingStates::kTraversalRung: {
            climber.TelescopingExtention(0.00);
            m_state = ClimbingStates::kManual;
>>>>>>> Stashed changes
            break;
        }
        case ClimbingStates::kManual: {
            break;
        }
        default: {
            m_state = ClimbingStates::kManual;
            break;
        }
    }  // Climbing State Machine End
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() {
    if constexpr (frc::RobotBase::IsSimulation()) {
        frc3512::DeleteCSVs();
    }
    return frc::StartRobot<frc3512::Robot>();
}
#endif
