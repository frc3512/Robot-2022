// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <cameraserver/CameraServer.h>

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

    // AUTONOMOUS MODES
    m_autonChooser.AddAutonomous("Drive Backwards", [=] { AutoBackwards(); });
    m_autonChooser.AddAutonomous("Shoot One", [=] { AutoShootOne(); });
    m_autonChooser.AddAutonomous("Shoot Two", [=] { AutoShootTwo(); });
    m_autonChooser.AddAutonomous("Shoot Three", [=] { AutoShootThree(); });
    m_autonChooser.AddAutonomous("Shoot Four", [=] { AutoShootFour(); });

    // TIMESLICE ALLOCATION TABLE
    //
    // |  Subsystem     | Duration (ms) | Allocation (ms) |
    // |----------------|---------------|-----------------|
    // | **Total**      | 5.0           | 5.0             |
    // | TimedRobot     | ?             | 2.0             |
    // | Drivetrain     | 1.35          | 1.5             |
    // | FrontFlywheel  | 0.6           | 0.7             |
    // | BackFlywheel   | 0.6           | 0.7             |
    // | **Free**       | None          | N/A             |
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
                frontFlywheel.ControllerPeriodic();
            }
        },
        0.7_ms);

    Schedule(
        [=] {
            if (IsEnabled()) {
                backFlywheel.ControllerPeriodic();
            }
        },
        0.7_ms);

    if constexpr (!IsSimulation()) {
        frc::CameraServer::StartAutomaticCapture();
    }

    vision.SubscribeToVisionData(drivetrain.visionQueue);
    vision.SubscribeToVisionData(backFlywheel.visionQueue);
    vision.SubscribeToVisionData(frontFlywheel.visionQueue);
}

Robot::~Robot() {}

units::second_t Robot::SelectedAutonomousDuration() const {
    return m_autonChooser.SelectedAutonomousDuration();
}

void Robot::SimulationInit() { SubsystemBase::RunAllSimulationInit(); }

void Robot::DisabledInit() {
    m_autonChooser.ResumeAutonomous();
    SubsystemBase::RunAllDisabledInit();

    // Reset teleop shooting state machine when disabling robot
    StopShooter();
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

    RunShooterSM();
    m_backFlywheelAtGoal.SetBoolean(backFlywheel.IsReady());
    m_frontFlywheelAtGoal.SetBoolean(frontFlywheel.IsReady());
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllTeleopPeriodic();
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};
    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    if (driveStick2.GetRawButton(5)) {
        drivetrain.AimWithVision();
    }

    if (frontFlywheel.IsReady() && backFlywheel.IsReady()) {
        if (driveStick1.GetRawButtonPressed(1) ||
            driveStick2.GetRawButtonPressed(1) ||
            driveStick2.GetRawButtonPressed(11)) {
            SetReadyToShoot(true);
        }
    } else {
        if (driveStick1.GetRawButtonPressed(1)) {
            Shoot(FrontFlywheelConstants::kShootHighTarmac,
                  BackFlywheelConstants::kShootHighTarmac);
        }
        if (driveStick2.GetRawButtonPressed(1)) {
            Shoot(FrontFlywheelConstants::kShootLow,
                  BackFlywheelConstants::kShootLow, true);
        }
        if (driveStick2.GetRawButtonPressed(11)) {
            Shoot(FrontFlywheelConstants::kShootHighFender,
                  BackFlywheelConstants::kShootHighFender);
        }
        if (driveStick2.GetRawButtonPressed(3)) {
            Shoot(frontFlywheel.GetGoalFromRange(),
                  backFlywheel.GetGoalFromRange());
        }
        if (driveStick1.GetRawButtonPressed(2)) {
            StopShooter();
        }
    }

    RunShooterSM();

    m_backFlywheelAtGoal.SetBoolean(backFlywheel.IsReady());
    m_frontFlywheelAtGoal.SetBoolean(frontFlywheel.IsReady());
}

void Robot::TestPeriodic() {
    SubsystemBase::RunAllTestPeriodic();
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    if (appendageStick1.GetRawButtonPressed(3)) {
        intake.SetTimeToShoot(true);
    }
    if (appendageStick1.GetRawButtonPressed(2)) {
        intake.SetTimeToShoot(false);
    }

    m_backFlywheelAtGoal.SetBoolean(backFlywheel.IsReady());
    m_frontFlywheelAtGoal.SetBoolean(frontFlywheel.IsReady());

    // RunShooterSM();
}

frc::Pose2d Robot::UpdateAutoPoseRotation(const frc::Pose2d& pose,
                                          units::radian_t newHeading) {
    return frc::Pose2d(pose.X(), pose.Y(), newHeading);
}

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

        EXPECT_EQ(frontFlywheel.GetGoal(), 0_rad_per_s);
        EXPECT_EQ(backFlywheel.GetGoal(), 0_rad_per_s);
    }
}

bool Robot::IsShooting() const { return m_state != ShootingState::kIdle; }

void Robot::Shoot(units::radians_per_second_t frontSpeed,
                  units::radians_per_second_t backSpeed, bool visionAim) {
    if (m_state == ShootingState::kIdle) {
        frontFlywheel.SetGoal(frontSpeed);
        backFlywheel.SetGoal(backSpeed);

        m_shootTimer.Reset();
        m_shootTimer.Stop();

        // if we don't want to aim with vision, skip ahead in the state machine.
        m_visionAim = visionAim;
        if (m_visionAim) {
            m_state = ShootingState::kVisionAim;
        } else {
            m_state = ShootingState::kSpinUp;
        }
    }
}

void Robot::RunShooterSM() {
    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    switch (m_state) {
        case ShootingState::kIdle:
            break;
        case ShootingState::kVisionAim:
            if (ReadyToShoot()) {
                drivetrain.AimWithVision();
                m_shootTimer.Start();
                m_state = ShootingState::kVisionSpinUp;
            }
            break;
        case ShootingState::kVisionSpinUp:
            if (drivetrain.AtHeading() ||
                (!drivetrain.AtHeading() && m_shootTimer.HasElapsed(3_s))) {
                frontFlywheel.SetGoal(frontFlywheel.GetGoalFromRange());
                backFlywheel.SetGoal(backFlywheel.GetGoalFromRange());
                drivetrain.SetHeadingGoal(drivetrain.GetAngle());
                m_state = ShootingState::kSpinUp;
            }
            break;
        case ShootingState::kSpinUp:
            if (ReadyToShoot()) {
                if (frc::RobotBase::IsAutonomousEnabled() &&
                    frontFlywheel.IsReady() && backFlywheel.IsReady()) {
                    m_state = ShootingState::kStartConveyor;
                } else if (frc::RobotBase::IsTeleopEnabled() &&
                           frontFlywheel.IsReady() && backFlywheel.IsReady()) {
                    m_state = ShootingState::kStartConveyor;
                }
            } else if (!frontFlywheel.IsReady() && !backFlywheel.IsReady() &&
                       m_shootTimer.HasElapsed(3_s)) {
                fmt::print(stderr, "Flywheels didn't get up to speed!");
                frontFlywheel.SetGoal(0_rad_per_s);
                backFlywheel.SetGoal(0_rad_per_s);
                m_shootTimer.Stop();
                m_state = ShootingState::kIdle;
            }
            break;
        case ShootingState::kStartConveyor:
            intake.SetTimeToShoot(true);
            m_shootTimer.Reset();
            m_shootTimer.Start();
            m_state = ShootingState::kFirstBall;
            break;
        case ShootingState::kFirstBall:
            if (m_shootTimer.HasElapsed(0.3_s)) {
                intake.SetTimeToShoot(false);
                m_shootTimer.Reset();
                m_state = ShootingState::kSecondBall;
            }
            break;
        case ShootingState::kSecondBall:
            if (m_shootTimer.HasElapsed(0.1_s)) {
                intake.SetTimeToShoot(true);
                m_shootTimer.Reset();
                m_state = ShootingState::kEndShoot;
            }
            break;
        case ShootingState::kEndShoot:
            if (m_shootTimer.HasElapsed(1_s)) {
                StopShooter();
                m_state = ShootingState::kIdle;
            }
            break;
    }
}

void Robot::StopShooter() {
    frontFlywheel.SetGoal(0_rad_per_s);
    backFlywheel.SetGoal(0_rad_per_s);
    intake.SetTimeToShoot(false);
    SetReadyToShoot(false);
    m_state = ShootingState::kIdle;
}

bool Robot::ReadyToShoot() const { return m_readyToShoot; }

void Robot::SetReadyToShoot(bool ready) { m_readyToShoot = ready; }

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() {
    if constexpr (frc::RobotBase::IsSimulation()) {
        frc3512::DeleteCSVs();
    }
    return frc::StartRobot<frc3512::Robot>();
}
#endif
