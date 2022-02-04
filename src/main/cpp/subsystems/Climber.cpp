// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <frc/Joystick.h>

#include "HWConfig.hpp"

using namespace frc3512;

Climber::Climber() {}

void Climber::TelescopingExtention(double yAxis) {
    m_leftTeleMotor.Set(yAxis);
    m_rightTeleMotor.Set(yAxis);
}

void Climber::TelescopingOut() {
    m_leftTeleSolenoid.Set(true);
    m_rightTeleSolenoid.Set(true);
}

void Climber::TelescopingIn() {
    m_leftTeleSolenoid.Set(false);
    m_rightTeleSolenoid.Set(false);
}

bool Climber::IsTelescopingOut() const { return m_rightTeleSolenoid.Get(); }

bool Climber::IsUpperSensorTriggered() const { return m_upperSensor.Get(); }

bool Climber::IsLowerSensorTriggered() const { return m_lowerSensor.Get(); }

bool Climber::IsBarSensorTriggered() const { return m_barSensor.Get(); }

void Climber::UpdateClimberSim(double extention) {
    if (IsTelescopingOut()) {
        m_climberSim->SetAngle(-180_deg);
    } else {
        m_climberSim->SetAngle(-90_deg);
    }

    double extentionLength = m_extentionBase->GetLength();

    extention += extentionLength;

    if (extention < -40) {
        extention = -40;
    }

    if (extention > -20) {
        extention = -20;
    }

    m_extentionBase->SetLength(extention);
}

void Climber::RobotPeriodic() {
    frc::SmartDashboard::PutData("Climber", &m_mech2d);
}

void Climber::SimulationPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::appendageStick1PortID};

    UpdateClimberSim(appendageStick1.GetRawAxis(1));
}
