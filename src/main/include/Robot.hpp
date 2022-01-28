// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <string_view>
#include <vector>

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/logging/CSVLogFile.h>
#include <units/time.h>

#include "AutonomousChooser.hpp"
#include "subsystems/Climber.hpp"
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
class Robot : public frc::TimedRobot {
public:
    Robot();
    ~Robot();
    /**
     * States used for climbing state machine
     */
    enum class ClimbingStates {
        kGround,
        kSecondRung,
        kThridRung,
        kTraversalRung
    };

    /// The Climber Subsystem
    Climber climber;

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
     * State Machine controlling the climbing sequence
     */
    void ClimbingSequenceSM();

private:
    ClimbingStates m_state = ClimbingStates::kGround;
    AutonomousChooser m_autonChooser{"No-op", [=] { AutoNoOp(); }};
};
}  // namespace frc3512
