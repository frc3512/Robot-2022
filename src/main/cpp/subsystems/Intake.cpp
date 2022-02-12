// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>

#include "CANSparkMaxUtil.hpp"

using namespace frc3512;
using namespace frc3512::HWConfig::Intake;

Intake::Intake() {
    SetCANSparkMaxBusUsage(m_leftConveyorMotor, Usage::kMinimal);
    m_leftConveyorMotor.SetSmartCurrentLimit(80);
    SetCANSparkMaxBusUsage(m_rightConveyorMotor, Usage::kMinimal);
    m_rightConveyorMotor.SetSmartCurrentLimit(80);
    SetCANSparkMaxBusUsage(m_intakeMotor, Usage::kMinimal);
    m_intakeMotor.SetSmartCurrentLimit(80);
}

void Intake::Deploy() { m_fourbar.Set(true); }

void Intake::Stow() { m_fourbar.Set(false); }

bool Intake::IsDeployed() const { return m_fourbar.Get(); }

void Intake::Start(IntakeDirection direction) {
    if (direction == IntakeDirection::kIntake) {
        m_intakeMotor.Set(0.8);
        SetConveyor(0.8);
    } else if (direction == IntakeDirection::kOuttake) {
        m_intakeMotor.Set(-0.8);
        SetConveyor(0.8);
    } else {
        m_intakeMotor.Set(0.0);
        SetConveyor(0.0);
    }
}

void Intake::Stop() {
    m_intakeMotor.Set(0.0);
    SetConveyor(0.0);
}

void Intake::SetConveyor(double speed) {
    m_leftConveyorMotor.Set(speed);
    m_rightConveyorMotor.Set(speed);
}

bool Intake::IsConveyorRunning() const {
    return m_leftConveyorMotor.Get() > 0.0 && m_rightConveyorMotor.Get() > 0.0;
}

bool Intake::IsUpperSensorBlocked() const { return m_upperSensor.Get(); }

bool Intake::IsLowerSensorBlocked() const { return m_lowerSensor.Get(); }

void Intake::RobotPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    // TODO: Write logic once the flywheel controller is merged

    frc::SmartDashboard::PutData("Intake", &m_intakeSim);

    m_upperSensorEntry.SetBoolean(IsUpperSensorBlocked());
    m_lowerSensorEntry.SetBoolean(IsLowerSensorBlocked());

    m_intakeLog.Log(m_fourbar.Get(), m_intakeMotor.Get());
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
