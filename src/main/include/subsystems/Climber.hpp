// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/filter/Debouncer.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <units/length.h>
#include <units/voltage.h>

#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {
/**
 * The climber is composed of two 'passive hooks' and two pivoting telescoping
 * arms.
 *
 * To climb, the elevator reaches for the 2nd climbing bar. Once picking itself
 * up, it deploys its two pivoting arms to reach for the next bar, which is
 * around ~2 feet up diagonally. Once stablized, it uses its passive arms to
 * hook onto the existing bars while the pivoting ones release and go for the
 * next bar, until we reach the last, most far up one.
 */
class Climber : public SubsystemBase {
public:
    /**
     * Climber states.
     */
    enum class ClimberState {
        kIdle,
        kReachedHeightLimit,
        kReadyToClimb,
        kRetracted
    };

    /**
     * Constructs a Climber.
     *
     */
    Climber();

    Climber(const Climber&) = delete;
    Climber& operator=(const Climber&) = delete;

    /**
     *  Deploys pivoting arm solenoids out
     */
    void DeployClimbers();

    /**
     *  Stows pivoting arm solenoids in
     */
    void StowClimbers();

    /**
     * Checks if pivoting arm solenoids are in
     */
    bool IsClimberDeployed();

    /**
     * Returns true if both climbers pass the upper limit
     */
    bool HasReachedUpperLimit();

    /**
     * Returns true if both climbers pass the bottom limit.
     */
    bool HasReachedBottomLimit();

    /**
     * Returns true when the system has determine the robot is ready to climb
     */
    bool IsReadyToClimb() const;

    /**
     * Returns the position of the left pivot arm.
     */
    units::meter_t GetLeftPivotPosition();

    /**
     * Returns the position of the right pivot arm.
     */
    units::meter_t GetRightPivotPosition();

    /**
     * Returns the voltage applied to the left pivot arm motor.
     */
    units::volt_t GetLeftPivotMotorOutput() const;

    /**
     * Returns the voltage applied to the right pivot arm motor.
     */
    units::volt_t GetRightPivotMotorOutput() const;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void SimulationPeriodic() override;

private:
    ClimberState state = ClimberState::kIdle;

    frc::DigitalInput m_leftInfraredSensorArmLeft{
        frc3512::HWConfig::Climber::kLeftArmLeftInfraredSensorChannel};
    frc::DigitalInput m_rightInfaredSensorArmLeft{
        frc3512::HWConfig::Climber::kLeftArmRightInfraredSensorChannel};

    frc::DigitalInput m_leftInfraredSensorArmRight{
        frc3512::HWConfig::Climber::kRightArmLeftInfraredSensorChannel};
    frc::DigitalInput m_rightInfaredSensorArmRight{
        frc3512::HWConfig::Climber::kRightArmRightInfraredSensorChannel};

    rev::CANSparkMax m_leftPivotArmMotor{
        frc3512::HWConfig::Climber::kLeftPivotingArmID,
        rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_rightPivotArmMotor{
        frc3512::HWConfig::Climber::kRightPivotingArmID,
        rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkMaxRelativeEncoder m_leftPivotArmEncoder =
        m_leftPivotArmMotor.GetEncoder();
    rev::SparkMaxRelativeEncoder m_rightPivotArmEncoder =
        m_rightPivotArmMotor.GetEncoder();

    frc::Solenoid m_leftPivotArmSolenoid{
        frc::PneumaticsModuleType::CTREPCM,
        frc3512::HWConfig::Climber::kLeftClimberLockChannel};
    frc::Solenoid m_rightPivotArmSolenoid{
        frc::PneumaticsModuleType::CTREPCM,
        frc3512::HWConfig::Climber::kRightClimberLockChannel};

    frc::Debouncer m_debouncer{50_ms, frc::Debouncer::DebounceType::kBoth};

    // Networktable entries
    nt::NetworkTableEntry m_leftPivotArmEncoderEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Left Pivoting Arm Encoder");

    nt::NetworkTableEntry m_rightPivotArmEncoderEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Right Pivoting Arm Encoder");

    nt::NetworkTableEntry m_leftInfraredSensorLeftArmEntry =
        NetworkTableUtil::MakeBoolEntry(
            "/Diagnostics/Climber/Left Arm Left Infrared Sensor");

    nt::NetworkTableEntry m_rightInfraredSensorLeftArmEntry =
        NetworkTableUtil::MakeBoolEntry(
            "/Diagnostics/Climber/Left Arm Right Infrared Sensor");

    nt::NetworkTableEntry m_leftInfraredSensorRightArmEntry =
        NetworkTableUtil::MakeBoolEntry(
            "/Diagnostics/Climber/Right Arm Left Infrared Sensor");

    nt::NetworkTableEntry m_rightInfraredSensorRightArmEntry =
        NetworkTableUtil::MakeBoolEntry(
            "/Diagnostics/Climber/Right Arm Right Infrared Sensor");

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_leftClimberSimLS{
        frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(), 4.5_kg,
                                            0.86_in, 20.0)};
    frc::sim::LinearSystemSim<2, 1, 1> m_rightClimberSimLS{
        frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(), 4.5_kg,
                                            0.86_in, 20.0)};

    frc::Mechanism2d m_climberSim{60, 60};
    frc::MechanismRoot2d* m_climberBase =
        m_climberSim.GetRoot("ClimberBase", 10, 20);
    frc::MechanismLigament2d* m_climberPartSim =
        m_climberBase->Append<frc::MechanismLigament2d>(
            "Climber", 20, -90_deg, 5, frc::Color8Bit{frc::Color::kBlue});
    frc::MechanismLigament2d* m_extensionBase =
        m_climberPartSim->Append<frc::MechanismLigament2d>(
            "Extended", -20, 0_deg, 5, frc::Color8Bit{frc::Color::kYellow});

    /* Sets the elevator speed.
     *
     * @param speed The speed of the elevator [-1..1].
     */
    void SetElevators(double speed);
};
}  // namespace frc3512
