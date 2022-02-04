// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include "HWConfig.hpp"

using namespace frc3512;

Intake::Intake() {}

void Intake::Deploy() { Intake::SetFourbar(FourbarDirection::kDeployed); }

void Intake::Stow() { Intake::SetFourbar(FourbarDirection::kStowed); }

bool Intake::IsDeployed() const { return m_fourbar.Get(); }

void Intake::Start() { SetFourbar(FourbarDirection::kDeployed); }

void Intake::Stop() { SetFourbar(FourbarDirection::kStowed); }

void Intake::Outtake() { SetFourbar(FourbarDirection::kDeployed); }

void Intake::SetConveyor(double speed) { m_conveyor.Set(speed); }

bool Intake::IsConveyorRunning() const { return m_conveyor.Get() != 0.0; }

void Intake::UpdateIntake() {
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
void Intake::RobotPeriodic() {
    frc::SmartDashboard::PutData("Intake", &m_mech2d);
}

void Intake::SimulationPeriodic() { UpdateIntake(); }
