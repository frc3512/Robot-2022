// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <string_view>
#include <vector>

#include <frc/Timer.h>
#include <frc/TimesliceRobot.h>
#include <frc/logging/CSVLogFile.h>
#include <units/time.h>

#include "AutonomousChooser.hpp"
#include "NetworkTableUtil.hpp"
#include "subsystems/BackFlywheel.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/FrontFlywheel.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/SubsystemBase.hpp"

#if RUNNING_FRC_TESTS
#include <gtest/gtest.h>
#else
namespace frc3512::testing {
/**
 * A stream object shim for the GoogleTest EXPECT macros.
 */
struct NoOp {
    /**
     * No-op.
     */
    void operator<<(const char*) {}
};
}  // namespace frc3512::testing
#define EXPECT_EQ(a, b) frc3512::testing::NoOp()
#define EXPECT_FALSE(a) frc3512::testing::NoOp()
#define EXPECT_GT(a, b) frc3512::testing::NoOp()
#define EXPECT_LT(a, b) frc3512::testing::NoOp()
#define EXPECT_NEAR(a, b, c) frc3512::testing::NoOp()
#define EXPECT_TRUE(a) frc3512::testing::NoOp()
#endif

namespace frc3512 {
/**
 * The main robot class.
 */
class Robot : public frc::TimesliceRobot {
public:
    /**
     * States used for the multi-subsystem shooting procedure
     */
    enum class ShootingState { kIdle, kSpinUp, kStartConveyor, kEndShoot };

    // The subsystem initialization order determines the controller run order.

    /// Drivetrain subsystem.
    Drivetrain drivetrain;

    /// Rear Flywheel subsystem.
    BackFlywheel backFlywheel;

    /// Front Flywheel subsystem.
    FrontFlywheel frontFlywheel;

    /// Intake subsystem.
    Intake intake;

    Robot();

    ~Robot();

    /**
     * Returns the selected autonomous mode's expected duration.
     */
    units::second_t SelectedAutonomousDuration() const;

    /**
     * Robot-wide simulation initialization code should go here.
     *
     * Users should override this method for default Robot-wide simulation
     * related initialization which will be called when the robot is first
     * started. It will be called exactly one time after RobotInit is called
     * only when the robot is in simulation.
     */
    void SimulationInit() override;

    /**
     * Initialization code for disabled mode should go here.
     */
    void DisabledInit() override;

    /**
     * Initialization code for autonomous mode should go here.
     */
    void AutonomousInit() override;

    /**
     * Initialization code for teleop mode should go here.
     */
    void TeleopInit() override;

    /**
     * Initialization coe for test mode should go here.
     */
    void TestInit() override;

    /**
     * Periodic code for all modes should go here.
     */
    void RobotPeriodic() override;

    /**
     * Periodic simulation code should go here.
     *
     * This function is called in a simulated robot after user code executes.
     */
    void SimulationPeriodic() override;

    /**
     * Periodic code for disabled mode should go here.
     */
    void DisabledPeriodic() override;

    /**
     * Periodic code for autonomous mode should go here.
     */
    void AutonomousPeriodic() override;

    /**
     * Periodic code for teleop mode should go here.
     */
    void TeleopPeriodic() override;

    /**
     * Periodic code for test mode should go here.
     */
    void TestPeriodic() override;

    /**
     * No-op autonomous.
     */
    void AutoNoOp();

    /**
     * Sets the selected autonomous mode for testing purposes.
     *
     * @param name The autonomous mode's name passed to
     *             AutonomousChooser::AddAutonomous().
     */
    void SelectAutonomous(std::string_view name);

    /**
     * Returns the names of autonomous modes to test.
     */
    const std::vector<std::string>& GetAutonomousNames() const;

    /**
     * Assertions to check at the end of each autonomous mode during unit
     * testing.
     */
    void ExpectAutonomousEndConds();

    /**
     * Returns whether or not the robot is shooting.
     */
    bool IsShooting() const;

    /**
     * Checks whether or not the driver wants to shoot high or low, then changes
     * the state of the state machine to execute shooting sequence.
     *
     * @param frontSpeed    Speed for the front flywheel
     * @param backSpeed     Speed for the back flywheel
     */
    void Shoot(units::radians_per_second_t frontSpeed,
               units::radians_per_second_t backSpeed);

    /**
     * Runs the shooter state machine.
     * When idle, the state machine does nothing, when the flywheel is starting
     * up, it checks to ensure that the flywheel gets up to speed. If so, then
     * the conveyor starts. If the flywheel isn't up to speed after a few
     * seconds, then the state machine idles. If successful the conveyor starts,
     * after a few seoncds (enough time for the robot two shoot the max two
     * balls we can carry), the conveyor and shooter turn off and the state
     * machine returns to idle.
     */
    void RunShooterSM();

    /**
     * Sets the shooter to a goal of the flywheels to 0 radians per second, and
     * sets the shooter state machine to idle.
     */
    void StopShooter();

    /**
     * Returns whether the driver has hit the shoot button again and is ready to
     * shoot
     */
    bool ReadyToShoot() const;

    /**
     * Sets whether the driver has clicked the shoot button again and is ready
     * to shoot the ball, not just spin up.
     */
    void SetReadyToShoot(bool ready);

private:
    frc::Timer m_shootTimer;
    ShootingState m_state = ShootingState::kIdle;
    bool m_readyToShoot = false;

    AutonomousChooser m_autonChooser{"No-op", [=] { AutoNoOp(); }};

    frc::CSVLogFile m_batteryLogger{"Battery", "Battery voltage (V)"};
    frc::CSVLogFile m_eventLogger{"Events", "Event"};

    nt::NetworkTableEntry m_batteryVoltageEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Robot/batteryVoltage");
};
}  // namespace frc3512
