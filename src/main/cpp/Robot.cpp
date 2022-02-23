// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <stdexcept>

#include <fmt/format.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/Joystick.h>
#include <frc/Notifier.h>
#include <frc/Processes.h>
#include <frc/RobotController.h>
#include <frc/Threads.h>
#include <frc/UidSetter.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "RealTimePriorities.hpp"
#include "Setup.hpp"
#include "logging/CSVUtil.hpp"

namespace frc3512 {

Robot::Robot() : frc::TimesliceRobot{2_ms, Constants::kControllerPeriod} {
    SetRTRuntimeLimit();

    {
        frc::UidSetter uidSetter{0};
        if (!frc::Notifier::SetHALThreadPriority(true,
                                                 kPrioHALNotifierThread)) {
            throw std::runtime_error(
                fmt::format("Giving HAL Notifier RT priority {} failed\n",
                            kPrioHALNotifierThread));
        }
    }

    if (!frc::SetProcessPriority("/usr/local/frc/bin/FRC_NetCommDaemon", true,
                                 kPrioNetCommDaemon)) {
        throw std::runtime_error(
            fmt::format("Giving /usr/local/frc/bin/FRC_NetCommDaemon RT "
                        "priority {} failed  ",
                        kPrioNetCommDaemon));
    }

    {
        frc::UidSetter uidSetter{0};
        if (!frc::SetCurrentThreadPriority(true, kPrioMainRobotThread)) {
            throw std::runtime_error(
                fmt::format("Giving TimesliceRobot RT priority {} failed\n",
                            kPrioMainRobotThread));
        }
    }

    StopCrond();

    // These warnings generate console prints that cause scheduling jitter
    frc::DriverStation::SilenceJoystickConnectionWarning(true);

    // This telemetry regularly causes loop overruns
    frc::LiveWindow::DisableAllTelemetry();

    // Log NT data every 20ms instead of every 100ms for higher resolution
    // dashboard plots
    SetNetworkTablesFlushEnabled(true);

    // TIMESLICE ALLOCATION TABLE
    //
    // |  Subsystem | Duration (ms) | Allocation (ms) |
    // |------------|---------------|-----------------|
    // | **Total**  | 5.0           | 5.0             |
    // | TimedRobot | ?             | 2.0             |
    // | Drivetrain | 1.32          | 1.5             |
    // | Flywheel   | 0.6           | 0.7             |
    // | Turret     | 0.6           | 0.8             |
    // | **Free**   | 0.0           | N/A             |
    Schedule(
        [=] {
            if (IsEnabled()) {
                drivetrain.ControllerPeriodic();
            }
        },
        1.5_ms);

    Schedule(
        [=] {
            if (IsEnabled()) {
                flywheel.ControllerPeriodic();
            }
        }, 0.7_ms
    );
}

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

void Robot::RobotPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();

    auto batteryVoltage = frc::RobotController::GetInputVoltage();
    m_batteryLogger.Log(
        units::second_t{std::chrono::steady_clock::now().time_since_epoch()},
        batteryVoltage);

    if (frc::DriverStation::IsDisabled() ||
        !frc::DriverStation::IsFMSAttached()) {
        m_batteryVoltageEntry.SetDouble(batteryVoltage);
    }
}

void Robot::SimulationPeriodic() { SubsystemBase::RunAllSimulationPeriodic(); }

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonChooser.ResumeAutonomous();
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllTeleopPeriodic();
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    if (flywheel.IsReady() && (appendageStick2.GetRawButtonPressed(1) ||
                               appendageStick1.GetRawButtonPressed(1))) {
        Shoot();
        m_timer.Start();
    }

    if (m_timer.HasElapsed(3_s))
    {
        flywheel.Stop();
        m_timer.Stop();
        m_timer.Reset();
    }
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

void Robot::Shoot() { intake.SetConveyor(0.2); }

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() {
    if constexpr (frc::RobotBase::IsSimulation()) {
        frc3512::DeleteCSVs();
    }
    return frc::StartRobot<frc3512::Robot>();
}
#endif
