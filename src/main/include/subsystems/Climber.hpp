// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {
/**
 * The Climber subsystem.
 *
 * The Climber consists of two telescoping arms-
 */
class Climber : public SubsystemBase {
public:
    Climber();
    /**
     *  Changes the extension of the telescoping arms
     *  by the giving y-Axis of a joystick
     */
    void TelescopingExtension(double speed);
    /**
     *  Deploys arm solenoids out
     */
    void TelescopingOut();
    /**
     *  Stows arm solenoids in
     */
    void TelescopingIn();
    /**
     *  Returns if the arms are out
     */
    bool IsTelescopingOut() const;
    /**
     * Returns if the climber encoder
     * reads it as over extending
     */
    bool IsOverExtended() const;
    /**
     * Returns if climber encoder
     * reads it as fully retracted
     */
    bool IsRetracted() const;
    /**
     * Updates Climber sim
     */
    void UpdateClimberSim(double extension);
    /**
     * Returns encoder value for the climber
     */
    units::meter_t GetClimberPosition() const;
    /**
     * Returns if bar sensor is triggered
     */
    bool IsBarSensorTriggered() const;
    
    void TeleopPeriodic() override;

    void RobotPeriodic() override;

    void SimulationPeriodic() override;

private:
    frc::Mechanism2d m_mech2d{60, 60};
    frc::MechanismRoot2d* m_climberBase =
        m_mech2d.GetRoot("ClimberBase", 10, 20);
    frc::MechanismLigament2d* m_climberSim =
        m_climberBase->Append<frc::MechanismLigament2d>(
            "Climber", 20, -90_deg, 5, frc::Color8Bit{frc::Color::kBlue});
    frc::MechanismLigament2d* m_extensionBase =
        m_climberSim->Append<frc::MechanismLigament2d>(
            "Extended", -20, 0_deg, 5, frc::Color8Bit{frc::Color::kYellow});

    rev::CANSparkMax m_climberSensor{frc3512::HWConfig::Climber::kClimbSensor,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkMaxRelativeEncoder m_climberEncoder =
        m_climberSensor.rev::CANSparkMax::GetEncoder(
            rev::SparkMaxRelativeEncoder::Type::kHallSensor,
            frc3512::HWConfig::Climber::kRevs);

    frc::DigitalInput m_barSensor{frc3512::HWConfig::Climber::kClimbSensor};

    rev::CANSparkMax m_leftTeleMotor{frc3512::HWConfig::Climber::kLeftTeleMotorID,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightTeleMotor{frc3512::HWConfig::Climber::kRightTeleMotorID,
                                      rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_leftTeleSolenoid{frc::PneumaticsModuleType::CTREPCM,
                                     frc3512::HWConfig::Climber::kLeftTeleSolenoid};

    frc::Solenoid m_rightTeleSolenoid{frc::PneumaticsModuleType::CTREPCM,
                                      frc3512::HWConfig::Climber::kRightTeleSolenoid};

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_climberSimLS{
        frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(), 4.5_kg,
                                            0.86_in, 20.0)};
};
}  // namespace frc3512
