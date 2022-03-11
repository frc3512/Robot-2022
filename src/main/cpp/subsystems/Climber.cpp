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
#include "HWConfig.hpp"

using namespace frc3512;
using namespace frc3512::HWConfig::Climber;

Climber::Climber() {
    SetCANSparkMaxBusUsage(m_leftGrbx, Usage::kPositionOnly);
    m_leftGrbx.SetSmartCurrentLimit(40);
    SetCANSparkMaxBusUsage(m_rightGrbx, Usage::kPositionOnly);
    m_rightGrbx.SetSmartCurrentLimit(40);

    SetCANSparkMaxBusUsage(m_rightGrbx, Usage::kPositionOnly);
    SetCANSparkMaxBusUsage(m_leftGrbx, Usage::kPositionOnly);
}

void Climber::DeployClimbers() { m_solenoid.Set(true); }

void Climber::StowClimbers() { m_solenoid.Set(false); }

bool Climber::IsClimberDeployed() { return m_solenoid.Get(); }

double Climber::GetLeftHeight() { return m_leftEncoder.GetPosition(); }

double Climber::GetRightHeight() { return m_rightEncoder.GetPosition(); }

bool Climber::HasRightPassedTopLimit() {
    if (IsClimberDeployed()) {
        return GetRightHeight() < -152;
    } else {
        return GetRightHeight() < -135;
    }
}

bool Climber::HasRightPassedBottomLimit() { return GetRightHeight() > 0.0; }

bool Climber::HasLeftPassedTopLimit() {
    if (IsClimberDeployed()) {
        return GetLeftHeight() < -152;
    } else {
        return GetLeftHeight() < -135;
    }
}

bool Climber::HasLeftPassedBottomLimit() { return GetLeftHeight() > 0.0; }

void Climber::RobotPeriodic() {
    frc::SmartDashboard::PutData("Climber", &m_climberSim);
}

void Climber::TeleopPeriodic() {
    // physically on to the left of the other controller, controls left climber
    // elevator.
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};
    // physically on the right of the other controller, controsl right climber
    // elevator.
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    double rightY = frc::ApplyDeadband(appendageStick1.GetRawAxis(1), 0.1);

    double leftY = frc::ApplyDeadband(appendageStick2.GetRawAxis(1), 0.1);

    // Disable soft limits for comps. Couldn't debug before matches, no one has
    // had any problems with them, so they're unnecessary for now.
    SetClimber(leftY, rightY, true);

    if (appendageStick1.GetRawButtonPressed(6)) {
        if (IsClimberDeployed()) {
            StowClimbers();
        } else {
            DeployClimbers();
        }
    }

    m_leftTopLimitEntry.SetBoolean(HasLeftPassedTopLimit());
    m_rightTopLimitEntry.SetBoolean(HasRightPassedTopLimit());
}

void Climber::TestPeriodic() {
    // physically on to the left of the other controller, controls left climber
    // elevator.
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};
    // physically on the right of the other controller, controsl right climber
    // elevator.
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    double rightY = frc::ApplyDeadband(appendageStick1.GetRawAxis(1), 0.1);

    double leftY = frc::ApplyDeadband(appendageStick2.GetRawAxis(1), 0.1);

    SetClimber(leftY, rightY, true);

    if (appendageStick1.GetRawButtonPressed(6)) {
        if (IsClimberDeployed()) {
            StowClimbers();
        } else {
            DeployClimbers();
        }
    }

    m_leftTopLimitEntry.SetBoolean(HasLeftPassedTopLimit());
    m_rightTopLimitEntry.SetBoolean(HasRightPassedTopLimit());
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

void Climber::SetClimber(double leftSpeed, double rightSpeed,
                         bool ignoreLimits) {
    if (ignoreLimits) {
        m_leftGrbx.Set(leftSpeed);
        m_rightGrbx.Set(rightSpeed);
    } else {
        if ((leftSpeed < 0.0 && !HasLeftPassedTopLimit()) ||
            (leftSpeed > 0.0 && !HasLeftPassedBottomLimit())) {
            m_leftGrbx.Set(leftSpeed);
        } else {
            m_leftGrbx.Set(0.0);
        }

        if ((rightSpeed < 0.0 && !HasRightPassedTopLimit()) ||
            (rightSpeed > 0.0 && !HasRightPassedBottomLimit())) {
            m_rightGrbx.Set(rightSpeed);
        } else {
            m_rightGrbx.Set(0.0);
        }
    }
}