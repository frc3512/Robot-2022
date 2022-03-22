// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>

#include "CANSparkMaxUtil.hpp"

using namespace frc3512;
using namespace frc3512::HWConfig::Intake;

Intake::Intake() {
    SetCANSparkMaxBusUsage(m_miniArmMotor, Usage::kMinimal);
    m_miniArmMotor.SetSmartCurrentLimit(80);
    SetCANSparkMaxBusUsage(m_conveyorMotor, Usage::kMinimal);
    m_conveyorMotor.SetSmartCurrentLimit(80);
    SetCANSparkMaxBusUsage(m_intakeMotor, Usage::kMinimal);
    m_intakeMotor.SetSmartCurrentLimit(80);

    // m_fourbar.Set(false);
}

void Intake::Deploy() { m_fourbar.Set(true); }

void Intake::Stow() { m_fourbar.Set(false); }

bool Intake::IsDeployed() const { return m_fourbar.Get(); }

void Intake::Start(IntakeDirection direction) {
    if (direction == IntakeDirection::kIntake) {
        m_intakeMotor.Set(0.95);
        m_miniArmMotor.Set(-0.95);
        m_state = IntakeDirection::kIntake;
    } else if (direction == IntakeDirection::kOuttake) {
        m_intakeMotor.Set(-0.95);
        m_miniArmMotor.Set(0.95);
        m_state = IntakeDirection::kOuttake;
    } else {
        m_intakeMotor.Set(0.0);
        m_miniArmMotor.Set(0.0);
        m_state = IntakeDirection::kIdle;
    }
}

void Intake::Stop() { Start(IntakeDirection::kIdle); }

void Intake::SetConveyor(double speed) { m_conveyorMotor.Set(-speed); }

bool Intake::IsConveyorRunning() const { return m_conveyorMotor.Get() > 0.0; }

bool Intake::IsUpperSensorBlocked() const { return !m_upperSensor.Get(); }

bool Intake::IsLowerSensorBlocked() const { return !m_lowerSensor.Get(); }

void Intake::SetTimeToShoot(bool timeToShoot) { m_timeToShoot = timeToShoot; }

bool Intake::IsTimeToShoot() const { return m_timeToShoot; }

void Intake::RobotPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (appendageStick2.GetRawButton(3)) {
        Start(IntakeDirection::kIntake);
    } else if (appendageStick2.GetRawButton(4)) {
        Start(IntakeDirection::kOuttake);
    } else {
        Stop();
    }

    if (appendageStick2.GetRawButtonPressed(1)) {
        if (IsDeployed()) {
            Stow();
        } else {
            Deploy();
        }
    }

    if (m_state == IntakeDirection::kOuttake) {
        SetConveyor(0.8);
    } else if (IsTimeToShoot()) {
        SetConveyor(-0.45);
    } else if (!IsTimeToShoot() && !IsUpperSensorBlocked() &&
               IsLowerSensorBlocked()) {
        SetConveyor(-0.8);
    } else {
        SetConveyor(0.0);
    }
}

void Intake::SimulationPeriodic() {
    if (IsDeployed()) {
        m_fourbarSim->SetAngle(0_deg);
    } else {
        m_fourbarSim->SetAngle(90_deg);
    }
}

void Intake::SetFourbar(FourbarDirection direction) {
    if (direction == FourbarDirection::kDeployed) {
        m_fourbar.Set(true);
    } else {
        m_fourbar.Set(false);
    }
}
