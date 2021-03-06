// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <wpi/numbers>

#include "CANSparkMaxUtil.hpp"
#include "Constants.hpp"
#include "HWConfig.hpp"

using namespace frc3512;
using namespace frc3512::HWConfig::Climber;

Climber::Climber() {
    SetCANSparkMaxBusUsage(m_leftGrbx, Usage::kPositionOnly);
    m_leftGrbx.SetSmartCurrentLimit(40);
    SetCANSparkMaxBusUsage(m_rightGrbx, Usage::kPositionOnly);
    m_rightGrbx.SetSmartCurrentLimit(40);
}

void Climber::DeployClimbers() { m_solenoid.Set(true); }

void Climber::StowClimbers() { m_solenoid.Set(false); }

bool Climber::IsClimberDeployed() { return m_solenoid.Get(); }

bool Climber::HasRightPassedTopLimit() {
    return (m_rightClimberSwitch.GetValue() < kSwitchConstant);
}

bool Climber::HasLeftPassedTopLimit() {
    return (m_leftClimberSwitch.GetValue() < kSwitchConstant);
}

units::volt_t Climber::GetLeftElevatorMotorOutput() const {
    return units::volt_t{m_leftGrbx.Get()};
}

units::volt_t Climber::GetRightElevatorMotorOutput() const {
    return units::volt_t{m_rightGrbx.Get()};
}

void Climber::RobotPeriodic() {
    frc::SmartDashboard::PutData("Climber", &m_climberSim);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_leftClimberSimLS.SetInput(Eigen::Vector<double, 1>{
            m_leftGrbx.Get() * frc::RobotController::GetInputVoltage()});
        m_rightClimberSimLS.SetInput(Eigen::Vector<double, 1>{
            m_rightGrbx.Get() * frc::RobotController::GetInputVoltage()});
        m_leftClimberSimLS.Update(20_ms);
        m_rightClimberSimLS.Update(20_ms);
    }
}

void Climber::TeleopPeriodic() {
    // physically on to the left of the other controller, controls left climber
    // elevator.
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};
    // physically on the right of the other controller, controsl right climber
    // elevator.
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    double rightY = frc::ApplyDeadband(appendageStick2.GetRawAxis(1),
                                       Constants::kJoystickDeadband) *
                    0.8;

    double leftY = frc::ApplyDeadband(appendageStick1.GetRawAxis(1),
                                      Constants::kJoystickDeadband) *
                   0.76;

    // Disable soft limits for comps. Couldn't debug before matches, no one has
    // had any problems with them, so they're unnecessary for now.
    SetClimber(leftY, rightY);

    if (appendageStick1.GetRawButtonPressed(1)) {
        if (IsClimberDeployed()) {
            StowClimbers();
        } else {
            DeployClimbers();
        }
    }

    if (appendageStick2.GetRawButtonPressed(2)) {
        m_ignoreLimits = !m_ignoreLimits;
    }

    m_leftTopSwitchEntry.SetDouble(m_leftClimberSwitch.GetValue());
    m_rightTopSwitchEntry.SetDouble(m_rightClimberSwitch.GetValue());
}

void Climber::TestPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    double rightY =
        frc::ApplyDeadband(appendageStick2.GetRawAxis(1), 0.1) * 0.76;

    double leftY = frc::ApplyDeadband(appendageStick1.GetRawAxis(1), 0.1) * 0.8;

    SetClimber(leftY, rightY);

    if (appendageStick2.GetRawButtonPressed(2)) {
        m_ignoreLimits = !m_ignoreLimits;
    }

    if (appendageStick1.GetRawButtonPressed(1)) {
        if (IsClimberDeployed()) {
            StowClimbers();
        } else {
            DeployClimbers();
        }
    }

    m_leftTopSwitchEntry.SetDouble(m_leftClimberSwitch.GetValue());
    m_rightTopSwitchEntry.SetDouble(m_rightClimberSwitch.GetValue());
}

void Climber::SimulationPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    double extension = appendageStick2.GetY();
    double extensionLength = m_extensionBase->GetLength();

    extension += extensionLength;

    if (extension < -40) {
        extension = -40;
    }

    if (extension > -20) {
        extension = -20;
    }

    m_extensionBase->SetLength(extension);
}

void Climber::SetClimber(double leftSpeed, double rightSpeed) {
    if (m_ignoreLimits) {
        m_leftGrbx.Set(leftSpeed);
        m_rightGrbx.Set(rightSpeed);
    } else {
        if (!HasLeftPassedTopLimit() || leftSpeed > 0) {
            m_leftGrbx.Set(leftSpeed);
        } else {
            m_leftGrbx.Set(0.0);
        }

        if (!HasRightPassedTopLimit() || rightSpeed > 0) {
            m_rightGrbx.Set(rightSpeed);
        } else {
            m_rightGrbx.Set(0.0);
        }
    }
}
