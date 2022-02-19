// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <units/time.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {
/**
 * The intake subsystem.
 *
 * The intake consists of intake rollers, attached to a fourbar for deployment.
 * It feeds to a conveyor above that.
 */
class Intake : public SubsystemBase {
public:
    Intake();

    Intake(const Intake&) = delete;
    Intake& operator=(const Intake&) = delete;

    /**
     *  Fourbar solenoid motor direction
     */
    enum class FourbarDirection { kDeployed, kStowed };

    /**
     * Intake motor direction
     */
    enum class IntakeDirection { kIdle, kIntake, kOuttake };

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
     *  Starts the intake rollers set to a certain direction
     *
     *  @param direction  Direction the intake to drive in (intake balls or
     * outake them)
     */
    void Start(IntakeDirection direction);

    /**
     *  Stops the intake rollers
     */
    void Stop();

    /**
     *  Sets the speed of the conveyor
     *
     *  @param speed    Speed of the conveyor motors
     */
    void SetConveyor(double speed);

    /**
     *  Returns weather or not the conveyor is running
     */
    bool IsConveyorRunning() const;

    /**
     * Returns whether or not the upper proximity sensor detects something; true
     * means something is detected, false means it is not.
     */
    bool IsUpperSensorBlocked() const;

    /**
     * Returns whether or not the lower proximity sensor detects something; true
     * means something is detected, false means it is not.
     */
    bool IsLowerSensorBlocked() const;

    void RobotPeriodic() override;

    void SimulationPeriodic() override;

private:
    frc::Timer m_conveyorTimer;

    rev::CANSparkMax m_intakeMotor{frc3512::HWConfig::Intake::kArmMotorID,
                                   rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_miniArmMotor{frc3512::HWConfig::Intake::kMiniArmMotorID,
                                    rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_conveyorMotor{
        frc3512::HWConfig::Intake::kConveyorMotorID,
        rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_fourbar{frc::PneumaticsModuleType::CTREPCM,
                            frc3512::HWConfig::Intake::kArmChannel};

    frc::DigitalInput m_upperSensor{
        frc3512::HWConfig::Intake::kUpperSensorChannel};
    frc::DigitalInput m_lowerSensor{
        frc3512::HWConfig::Intake::kLowerSensorChannel};

    frc::Mechanism2d m_intakeSim{60, 60};
    frc::MechanismRoot2d* m_intakeBase =
        m_intakeSim.GetRoot("IntakeBase", 10, 00);
    frc::MechanismLigament2d* m_fourbarSim =
        m_intakeBase->Append<frc::MechanismLigament2d>(
            "Intake", 15, -90_deg, 5, frc::Color8Bit{frc::Color::kBlue});

    nt::NetworkTableEntry m_upperSensorEntry = NetworkTableUtil::MakeBoolEntry(
        "/Diagnostics/Intake/Upper Sensor blocked");
    nt::NetworkTableEntry m_lowerSensorEntry = NetworkTableUtil::MakeBoolEntry(
        "/Diagnostics/Intake/Lower Sensor blocked");

    frc::CSVLogFile m_intakeLog{"Intake", "Deployed (bool)", "Speed (-1 .. 1)"};

    /*
     *  Sets the fourbar to deployed or stowed
     */
    void SetFourbar(FourbarDirection direction);
};
}  // namespace frc3512
