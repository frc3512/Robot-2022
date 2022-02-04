// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
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
    Climber(const Climber&) = delete;
    Climber& operator=(const Climber&) = delete;

    /**
     *  Changes the extention of the telescoping arms
     *  by the giving y-Axis of a joystick
     */
    void TelescopingExtention(double speed);
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
     * Returns if the upper sensor
     * detects anything
     */
    bool IsUpperSensorTriggered() const;
    /**
     * Returns if the lower sensor
     * detects anything
     */
    bool IsLowerSensorTriggered() const;
    /**
     * Returns if the bar sensor
     * detects anything
     */
    bool IsBarSensorTriggered() const;
    /**
     * Updates Climber sim
     */
    void UpdateClimberSim(double extention);

    void RobotPeriodic() override;

    void SimulationPeriodic() override;

private:
    frc::Mechanism2d m_mech2d{60, 60};
    frc::MechanismRoot2d* m_climberBase =
        m_mech2d.GetRoot("ClimberBase", 10, 20);
    frc::MechanismLigament2d* m_climberSim =
        m_climberBase->Append<frc::MechanismLigament2d>(
            "Climber", 20, -90_deg, 5, frc::Color8Bit{frc::Color::kBlue});
    frc::MechanismLigament2d* m_extentionBase =
        m_climberSim->Append<frc::MechanismLigament2d>(
            "Extended", -20, 0_deg, 5, frc::Color8Bit{frc::Color::kYellow});

    frc::DigitalInput m_upperSensor{frc3512::HWConfig::upperSensorID};
    frc::DigitalInput m_lowerSensor{frc3512::HWConfig::lowerSensorID};
    frc::DigitalInput m_barSensor{frc3512::HWConfig::barSensorID};

    rev::CANSparkMax m_leftTeleMotor{frc3512::HWConfig::kLeftTeleMotorID,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightTeleMotor{frc3512::HWConfig::kRightTeleMotorID,
                                      rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_leftTeleSolenoid{frc::PneumaticsModuleType::CTREPCM,
                                     frc3512::HWConfig::kLeftTeleSolenoid};

    frc::Solenoid m_rightTeleSolenoid{frc::PneumaticsModuleType::CTREPCM,
                                      frc3512::HWConfig::kRightTeleSolenoid};
};
}  // namespace frc3512
