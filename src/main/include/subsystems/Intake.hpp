// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {
/**
 * The intake subsystem.
 *
 * The intake consists of intake rollers and a conveyor above that.
 */
class Intake : public SubsystemBase {
public:
    /**
     *  Fourbar solenoid motor direction
     */
    enum class FourbarDirection { kDeployed, kStowed };
    /**
     *  Deploys the intake
     */
    void Deploy();
    /**
     *  Stows the intake
     */
    void Stow();
    /**
     *  Returns wheather or not the intake is deployed
     */
    bool IsDeployed() const;
    /**
     *  Starts the intake rollers set to intake
     */
    void Start();
    /**
     *  Stops the intake rollers
     */
    void Stop();
    /**
     *  Sets the intake rollers to outtake
     */
    void Outtake();
    /**
     *  Sets the coneyor speed
     */
    void SetConveyor(double speed);
    /**
     *  Returns weather or not the conveyor is running
     */
    bool IsConveyorRunning() const;

    void RobotPeriodic() override;

private:
    rev::CANSparkMax m_intakeMotor{frc3512::HWConfig::kIntakeMotorID,
                                   rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_fourbar{frc::PneumaticsModuleType::CTREPCM,
                            frc3512::HWConfig::kFourbarChannel};

    rev::CANSparkMax m_conveyor{frc3512::HWConfig::kConveyorMotorID,
                                rev::CANSparkMax::MotorType::kBrushless};
    /*
     *  Sets the fourbar to deployed or stowed
     */
    void SetFourbar(FourbarDirection direction);
};
}  // namespace frc3512
