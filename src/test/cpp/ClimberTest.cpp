// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/Timer.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Climber.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs(val1 - val2), eps)

class ClimberTest : public frc3512::SimulatorTest {
public:
    frc3512::Climber climber;
};

TEST_F(ClimberTest, ConfigSpaceLimits) {
    frc::Timer timer;
    timer.Start();

    frc::sim::JoystickSim appendageStick1{
        frc3512::HWConfig::kAppendageStick1Port};

    frc::sim::JoystickSim appendageStick2{
        frc3512::HWConfig::kAppendageStick2Port};

    frc3512::SubsystemBase::RunAllTeleopInit();
    frc::Notifier controllerPeriodic{[&] {
        climber.RobotPeriodic();
        climber.TeleopPeriodic();
    }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    // Move right climber into top limit
    while (!climber.HasRightPassedTopLimit()) {
        appendageStick1.SetY(1.0);
        appendageStick1.NotifyNewData();
        frc::sim::StepTiming(20_ms);

        ASSERT_LT(timer.Get(), 30_s)
            << "Climber took too long to reach top limit";
    }

    // Don't let right climber move past top limit
    appendageStick1.SetY(1.0);
    appendageStick1.NotifyNewData();
    frc::sim::StepTiming(20_ms);
    EXPECT_EQ(climber.GetRightElevatorMotorOutput(), 0_V);

    frc::sim::StepTiming(1_s);

    // Move left climber into top limit
    while (!climber.HasLeftPassedTopLimit()) {
        appendageStick2.SetY(1.0);
        appendageStick2.NotifyNewData();
        frc::sim::StepTiming(20_ms);

        ASSERT_LT(timer.Get(), 30_s)
            << "Climber took too long to reach top limit";
    }

    // Don't let left climber move past top limit
    appendageStick2.SetY(1.0);
    appendageStick2.NotifyNewData();
    frc::sim::StepTiming(20_ms);
    EXPECT_EQ(climber.GetRightElevatorMotorOutput(), 0_V);
}
