// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
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
}

void Climber::DeployClimbers() { m_solenoid.Set(false); }

void Climber::StowClimbers() { m_solenoid.Set(true); }

bool Climber::IsClimberDeployed() { return !m_solenoid.Get(); }

units::meter_t Climber::GetLeftHeight() {
    double kG = 1.0 / 12.0;  // gear ratio

    if constexpr (frc::RobotBase::IsSimulation()) {
        return units::meter_t{m_leftClimberSimLS.GetOutput(0)};
    } else {
        double rotations = -m_leftEncoder.GetPosition();
        return units::meter_t{
            0.955 * wpi::numbers::pi * kG * rotations /
            (1.0 + 0.20675 * wpi::numbers::pi * kG * rotations)};
    }
}

units::meter_t Climber::GetRightHeight() {
    double kG = 1.0 / 12.0;  // gear ratio

    if constexpr (frc::RobotBase::IsSimulation()) {
        return units::meter_t{m_rightClimberSimLS.GetOutput(0)};
    } else {
        double rotations = -m_rightEncoder.GetPosition();
        return units::meter_t{
            0.955 * wpi::numbers::pi * kG * rotations /
            (1.0 + 0.20675 * wpi::numbers::pi * kG * rotations)};
    }
}

bool Climber::HasRightPassedTopLimit() {
    if (IsClimberDeployed()) {
        return GetRightHeight() > 0.711_m;
    } else {
        return GetRightHeight() > 0.66_m;
    }
}

bool Climber::HasRightPassedBottomLimit() { return GetRightHeight() < 0_m; }

bool Climber::HasLeftPassedTopLimit() {
    if (IsClimberDeployed()) {
        return GetLeftHeight() > 0.711_m;
    } else {
        return GetLeftHeight() > 0.66_m;
    }
}

bool Climber::HasLeftPassedBottomLimit() { return GetLeftHeight() < 0_m; }

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

    double rightY = appendageStick1.GetRawAxis(1) * 0.75;

    double leftY = appendageStick2.GetRawAxis(1) * 0.75;

    SetClimber(leftY, rightY);

    if (appendageStick2.GetRawButtonPressed(11)) {
        DeployClimbers();
    }
    if (appendageStick2.GetRawButtonPressed(12)) {
        StowClimbers();
    }
}

void Climber::TestPeriodic() {}

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
    if ((leftSpeed > 0.0 && !HasLeftPassedTopLimit()) ||
        (leftSpeed < 0.0 && !HasLeftPassedBottomLimit())) {
        m_leftGrbx.Set(leftSpeed);
    } else {
        m_leftGrbx.Set(0.0);
    }

    if ((rightSpeed > 0.0 && !HasRightPassedTopLimit()) ||
        (rightSpeed < 0.0 && !HasRightPassedBottomLimit())) {
        m_rightGrbx.Set(rightSpeed);
    } else {
        m_rightGrbx.Set(rightSpeed);
    }
}
