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

    if (appendageStick1.GetRawButtonPressed(4)) {
        intake.Deploy();
    }

    if (appendageStick1.GetRawButtonPressed(5)) {
        intake.Stow();
    }

    if (appendageStick1.GetRawButtonPressed(2)) {
        climber.TelescopingIn();
    }

    if (appendageStick1.GetRawButtonPressed(3)) {
        climber.TelescopingOut();
    }

    climber.TelescopingExtention(appendageStick1.GetRawAxis(1));

    if (appendageStick1.GetRawButtonPressed(1)) {
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

        EXPECT_TRUE(drivetrain.AtGoal());

        // Verify left/right wheel velocities are close to zero
        EXPECT_NEAR(
            drivetrain.GetStates()(DrivetrainController::State::kLeftVelocity),
            0.0, 0.01);
        EXPECT_NEAR(
            drivetrain.GetStates()(DrivetrainController::State::kRightVelocity),
            0.0, 0.01);
    }
}

void Robot::ClimbingSequenceSM() {
    switch (m_state) {
        case ClimbingStates::kSecondRung: {
            ClimbingSequence();

            break;
        }
        case ClimbingStates::kThridRung: {
            ClimbingSequence();
            break;
        }
        case ClimbingStates::kTraversalRung: {
            m_state = ClimbingStates::kManual;
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

void Robot::ClimbingSequence() {
    solenoidTimer.Start();
    if (solenoidTimer.HasElapsed(1_s)) {
        climber.TelescopingOut();
    } else if ((!climber.IsBarSensorTriggered()) &&
               (!climber.IsUpperSensorTriggered())) {
        climber.TelescopingExtention(0.65);
    }

    if (climber.IsUpperSensorTriggered()) {
        climber.TelescopingExtention(0.00);
    }

    if ((climber.IsBarSensorTriggered()) &&
        (!climber.IsLowerSensorTriggered())) {
        climber.TelescopingExtention(-0.65);
    }

    if (climber.IsLowerSensorTriggered()) {
        climber.TelescopingExtention(0.00);
    }

    if ((climber.IsLowerSensorTriggered()) && (solenoidTimer.HasElapsed(8_s))) {
        climber.TelescopingIn();
        solenoidTimer.Stop();
        solenoidTimer.Reset();

        if (m_state == ClimbingStates::kSecondRung) {
            m_state = ClimbingStates::kThridRung;
        } else {
            m_state = ClimbingStates::kTraversalRung;
        }
    }
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
