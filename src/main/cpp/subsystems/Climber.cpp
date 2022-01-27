// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include "HWConfig.hpp"

using namespace frc3512;

void Climber::TelescopingExtention(double yAxis) {
    if (HWConfig::kTEMPSensorVal) {
        m_leftTeleMotor.Set(yAxis);
        m_rightTeleMotor.Set(yAxis);
    }
}

void Climber::TeleopPeriodic() {}

void Climber::TelescopingOut() {
    m_leftTeleSolenoid.Set(true);
    m_rightTeleSolenoid.Set(true);
}

void Climber::TelescopingIn() {
    m_leftTeleSolenoid.Set(false);
    m_rightTeleSolenoid.Set(false);
}

bool Climber::IsTelescopingOut() const { return m_rightTeleSolenoid.Get(); }
