// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/filter/Debouncer.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
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
     * Returns whether or not the climber is fully extended when the
     * climber is deployed.
     */
    bool HasReachedFullUpperLimit();

    /**
     * Returns true if both climbers pass the bottom limit.
     */
    bool HasReachedBottomLimit();

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void SimulationPeriodic() override;

private:
    ClimberState state = ClimberState::kIdle;

    frc::DigitalInput m_extendedSensor{HWConfig::Climber::kTopSensorChannel};
    frc::DigitalInput m_retractedSensor{
        HWConfig::Climber::kBottomSensorChannel};

    rev::CANSparkMax m_leftArmMotor{HWConfig::Climber::kLeftClimberID,
                                    rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightArmMotor{HWConfig::Climber::kRightClimberID,
                                     rev::CANSparkMax::MotorType::kBrushless};
    frc::MotorControllerGroup m_climber{m_leftArmMotor, m_rightArmMotor};

    frc::Solenoid m_solenoid{frc::PneumaticsModuleType::CTREPCM,
                             HWConfig::Climber::kClimberSolenoidChannel};

    frc::Debouncer m_debouncer{50_ms, frc::Debouncer::DebounceType::kBoth};

    nt::NetworkTableEntry m_climberSolenoidEntry =
        NetworkTableUtil::MakeBoolEntry(
            "Diagnostics/Climber/Output/Climber Deployed");
    nt::NetworkTableEntry m_upperSensorEntry = NetworkTableUtil::MakeBoolEntry(
        "Diagnostics/Climber/Output/Climber Fully Extended");
    nt::NetworkTableEntry m_lowerSensorEntry = NetworkTableUtil::MakeBoolEntry(
        "Diagnostics/Climber/Output/Climber Fully Retracted");

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

    /**
     *  Sets the speed of the climber motor controller group.
     *
     * @param speed the speed of the flywheel.
     */
    void SetClimber(double speed);
};
}  // namespace frc3512
