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
    SetCANSparkMaxBusUsage(m_leftPivotArmMotor, Usage::kPositionOnly);
    m_leftPivotArmMotor.SetSmartCurrentLimit(40);
    SetCANSparkMaxBusUsage(m_rightPivotArmMotor, Usage::kPositionOnly);
    m_rightPivotArmMotor.SetSmartCurrentLimit(40);
}

void Climber::DeployClimbers() {
    m_leftPivotArmSolenoid.Set(true);
    m_rightPivotArmSolenoid.Set(true);
}

void Climber::StowClimbers() {
    m_leftPivotArmSolenoid.Set(false);
    m_rightPivotArmSolenoid.Set(false);
}

bool Climber::IsClimberDeployed() { return m_leftPivotArmSolenoid.Get(); }

bool Climber::HasReachedUpperLimit() {
    return (GetLeftPivotPosition() > 1.1129_m &&
            GetRightPivotPosition() > 1.1129_m);
}

bool Climber::HasReachedBottomLimit() {
    return (GetLeftPivotPosition() < 0_m && GetRightPivotPosition() < 0_m);
}

bool Climber::IsReadyToClimb() const {
    return (!m_leftInfraredSensorArmLeft.Get() &&
            m_rightInfaredSensorArmRight.Get()) &&
           (!m_leftInfraredSensorArmRight.Get() &&
            m_rightInfaredSensorArmRight.Get());
}

units::meter_t Climber::GetLeftPivotPosition() {
    constexpr double kG = 1.0 / 12.0;  // Gear ratio

    if constexpr (frc::RobotBase::IsSimulation()) {
        return units::meter_t{m_leftClimberSimLS.GetOutput(0)};
    } else {
        double rotations = -m_leftPivotArmEncoder.GetPosition();
        return units::meter_t{
            0.04381 * wpi::numbers::pi * kG * rotations /
            (1.0 + 0.014983 * wpi::numbers::pi * kG * rotations)};
    }
}

units::meter_t Climber::GetRightPivotPosition() {
    constexpr double kG = 1.0 / 12.0;  // Gear ratio

    if constexpr (frc::RobotBase::IsSimulation()) {
        return units::meter_t{m_rightClimberSimLS.GetOutput(0)};
    } else {
        double rotations = -m_rightPivotArmEncoder.GetPosition();
        return units::meter_t{
            0.04381 * wpi::numbers::pi * kG * rotations /
            (1.0 + 0.014983 * wpi::numbers::pi * kG * rotations)};
    }
}

units::volt_t Climber::GetLeftPivotMotorOutput() const {
    return units::volt_t{-m_leftPivotArmMotor.Get()};
}

units::volt_t Climber::GetRightPivotMotorOutput() const {
    return units::volt_t{-m_rightPivotArmMotor.Get()};
}

void Climber::RobotPeriodic() {
    frc::SmartDashboard::PutData("Climber", &m_climberSim);

    m_leftPivotArmEncoderEntry.SetDouble(m_leftPivotArmMotor.Get());
    m_rightPivotArmEncoderEntry.SetDouble(m_rightPivotArmMotor.Get());

    m_leftInfraredSensorLeftArmEntry.SetBoolean(
        m_leftInfraredSensorArmLeft.Get());
    m_rightInfraredSensorLeftArmEntry.SetBoolean(
        m_rightInfaredSensorArmLeft.Get());

    m_leftInfraredSensorRightArmEntry.SetBoolean(
        m_leftInfraredSensorArmRight.Get());
    m_rightInfraredSensorRightArmEntry.SetBoolean(
        m_rightInfaredSensorArmRight.Get());
}

void Climber::TeleopPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (appendageStick2.GetRawButton(11)) {
        SetElevators(appendageStick2.GetY());
    } else {
        SetElevators(0.0);
    }

    if (!IsClimberDeployed() && appendageStick2.GetRawButton(6)) {
        DeployClimbers();
    } else if (appendageStick2.GetRawButton(6)) {
        StowClimbers();
    }
}

void Climber::TestPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    // Positive voltage should move climber in the positive X direction
    double speed = -appendageStick2.GetY();

    // Ignore soft limits so the user can manually reset the elevator before
    // rebooting the robot
    if (std::abs(speed) > 0.02) {
        SetElevators(-speed);
    } else {
        SetElevators(0.0);
    }
}

void Climber::SimulationPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (!HasReachedBottomLimit()) {
        m_climberPartSim->SetAngle(-180_deg);
    } else {
        m_climberPartSim->SetAngle(-90_deg);
    }

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

void Climber::SetElevators(double speed) {
    if ((speed > 0.02 && !HasReachedUpperLimit()) ||
        (speed < -0.02 && !HasReachedUpperLimit())) {
        // Unlock climber if it's being commanded to move
        DeployClimbers();
        m_leftPivotArmMotor.Set(-speed);
        m_rightPivotArmMotor.Set(-speed);
    } else {
        StowClimbers();
        m_leftPivotArmMotor.Set(0.0);
        m_rightPivotArmMotor.Set(0.0);
    }
}
