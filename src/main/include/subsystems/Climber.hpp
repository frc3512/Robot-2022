// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
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
    bool UppperSensor() const;
    /**
     * Returns if the lower sensor
     * detects anything
     */
    bool LowerSensor() const;
    /**
     * Returns if the bar sensor
     * detects anything
     */
    bool BarSensor() const;

    void TeleopPeriodic() override;

private:
    frc::DigitalInput upperSensor{frc3512::HWConfig::upperSensorID};
    frc::DigitalInput lowerSensor{frc3512::HWConfig::lowerSensorID};
    frc::DigitalInput barSensor{frc3512::HWConfig::barSensorID};

    rev::CANSparkMax m_leftTeleMotor{frc3512::HWConfig::kLeftTeleMotorID,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightTeleMotor{frc3512::HWConfig::kRightTeleMotorID,
                                      rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_leftTeleSolenoid{frc::PneumaticsModuleType::CTREPCM,
                                     frc3512::HWConfig::kLeftTeleSolenoid};

    frc::Solenoid m_rightTeleSolenoid{frc::PneumaticsModuleType::CTREPCM,
                                      frc3512::HWConfig::kLeftTeleSolenoid};
};
}  // namespace frc3512
