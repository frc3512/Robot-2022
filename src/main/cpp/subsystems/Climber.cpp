// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <wpi/MathExtras.h>
#include <wpi/numbers>

#include "HWConfig.hpp"

using namespace frc3512;

Climber::Climber() {}

void Climber::TelescopingExtension(double yAxis) {
    
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

bool Climber::IsOverExtended() const {
    if (GetClimberPosition() > 0.1_m) {
        return true;
    } else {
        return false;
    }
}

bool Climber::IsRetracted() const {
    if (GetClimberPosition() == 0.0_m) {
        return true;
    } else {
        return false;
    }
}

void Climber::UpdateClimberSim(double extension) {
    if (IsTelescopingOut()) {
        m_climberSim->SetAngle(-180_deg);
    } else {
        m_climberSim->SetAngle(-90_deg);
    }

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

units::meter_t Climber::GetClimberPosition() const {
    constexpr double kG = 1.0 / 20.0;  // Gear ratio
    if constexpr (frc::RobotBase::IsSimulation()) {
        return units::meter_t{m_climberSimLS.GetOutput(0)};
    } else {
        double rotations = -m_climberEncoder.GetPosition();
        return units::meter_t{
            0.04381 * wpi::numbers::pi * kG * rotations /
            (1.0 + 0.014983 * wpi::numbers::pi * kG * rotations)};
    }
}

bool Climber::IsBarSensorTriggered() const { return m_barSensor.Get(); }

void Climber::TeleopPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    if (appendageStick1.GetRawButtonPressed(2)) {
        TelescopingIn();
    }

    if (appendageStick1.GetRawButtonPressed(3)) {
        TelescopingOut();
    }

    TelescopingExtension(-appendageStick1.GetY());
}

void Climber::RobotPeriodic() {
    frc::SmartDashboard::PutData("Climber", &m_mech2d);
}

void Climber::SimulationPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    UpdateClimberSim(appendageStick1.GetRawAxis(1));
}
