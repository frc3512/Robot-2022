// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/BackFlywheel.hpp"
#include "subsystems/FrontFlywheel.hpp"

class FlywheelTest : public frc3512::SimulatorTest {
public:
    frc3512::FrontFlywheel frontFlywheel;
    frc3512::BackFlywheel backFlywheel;
    frc::Notifier controllerPeriodic{[&] {
        frontFlywheel.TeleopPeriodic();
        backFlywheel.TeleopPeriodic();
        frontFlywheel.ControllerPeriodic();
        backFlywheel.ControllerPeriodic();
    }};

    FlywheelTest() {
        frc3512::SubsystemBase::RunAllTeleopInit();
        controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);
    }
};

TEST_F(FlywheelTest, ReachesGoal) {
    frontFlywheel.SetGoal(500_rad_per_s);
    backFlywheel.SetGoal(500_rad_per_s);

    frc::sim::StepTiming(2_s);

    EXPECT_TRUE(frontFlywheel.AtGoal());
    EXPECT_TRUE(backFlywheel.AtGoal());
}
