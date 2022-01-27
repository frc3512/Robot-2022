#pragma once

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
    bool IsTelescopingOut();

    void TeleopPeriodic() override;

private:
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