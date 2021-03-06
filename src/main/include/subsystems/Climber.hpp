// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/AnalogInput.h>
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
    /// Magnetic Switch constant value`
    const int kSwitchConstant = 3000;
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
     * Returns whether or not the right climber has passed the top limit
     */
    bool HasRightPassedTopLimit();

    /**
     * Returns whether or not the left climber has passed the top limit
     */
    bool HasLeftPassedTopLimit();

    /**
     * Returns the voltage applied to left elevator motor.
     */
    units::volt_t GetLeftElevatorMotorOutput() const;

    /**
     * Returns the voltage applied to right elevator motor.
     */
    units::volt_t GetRightElevatorMotorOutput() const;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void SimulationPeriodic() override;

private:
    frc::AnalogInput m_leftClimberSwitch{
        HWConfig::Climber::kLeftMagneticSwitch};
    frc::AnalogInput m_rightClimberSwitch{
        HWConfig::Climber::kRightMagnticSwitch};

    rev::CANSparkMax m_leftGrbx{HWConfig::Climber::kLeftClimberID,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightGrbx{HWConfig::Climber::kRightClimberID,
                                 rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_solenoid{frc::PneumaticsModuleType::CTREPCM,
                             HWConfig::Climber::kClimberSolenoidChannel};

    frc::Debouncer m_debouncer{50_ms, frc::Debouncer::DebounceType::kBoth};

    bool m_ignoreLimits = false;

    // Networktable entries
    nt::NetworkTableEntry m_leftElevatorEncoderEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Left Elevator Encoder");

    nt::NetworkTableEntry m_rightElevatorEncoderEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Right Elevator Encoder");

    nt::NetworkTableEntry m_leftTopSwitchEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Left Top Switch");
    nt::NetworkTableEntry m_rightTopSwitchEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Right Top Switch");
    nt::NetworkTableEntry m_leftArmEntry = NetworkTableUtil::MakeDoubleEntry(
        "/Diagnostics/Climber/Left Arm Value");
    nt::NetworkTableEntry m_rightArmEntry = NetworkTableUtil::MakeDoubleEntry(
        "/Diagnostics/Climber/Right Arm Value");

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_leftClimberSimLS{
        frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(), 0.5_kg,
                                            0.125_in, 12.0)};
    frc::sim::LinearSystemSim<2, 1, 1> m_rightClimberSimLS{
        frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(), 0.5_kg,
                                            0.125_in, 12.0)};

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
     *  Sets the speed of the individual climber motors.
     *
     * @param leftSpeed left side climber speed.
     * @param rightSpeed right side climber speed.
     * @param ignoreLimits whether or not to ignore soft limits on climber.
     */
    void SetClimber(double leftSpeed, double righSpeed);
};
}  // namespace frc3512
