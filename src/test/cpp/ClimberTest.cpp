// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Climber.hpp"

class ClimberTest : public frc3512::SimulatorTest {};

TEST_F(ClimberTest, Extension) {
    frc::sim::JoystickSim appendageStick1{
        frc3512::HWConfig::kAppendageStick1Port};

    frc3512::Climber climber;

    frc3512::SubsystemBase::RunAllTeleopPeriodic();
    frc::Notifier controllerPeriodic{[&] {
        climber.TeleopPeriodic();
    }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    appendageStick1.SetY(1.00);
    frc::sim::StepTiming(15_s);
    appendageStick1.NotifyNewData();
    frc3512::SubsystemBase::RunAllTeleopPeriodic();


    EXPECT_TRUE(climber.IsOverExtended());

    appendageStick1.SetY(-1.00);
    frc::sim::StepTiming(15_s);
    appendageStick1.NotifyNewData();
    frc3512::SubsystemBase::RunAllTeleopPeriodic();


    EXPECT_TRUE(climber.IsRetracted());
}

TEST_F(ClimberTest, Telescoping_out) {
    frc::sim::JoystickSim appendageStick1{
        frc3512::HWConfig::kAppendageStick1Port};

    frc3512::Climber climber;

    frc3512::SubsystemBase::RunAllTeleopPeriodic();
    frc::Notifier controllerPeriodic{[&] {
        climber.TeleopPeriodic();
    }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    appendageStick1.SetRawButton(3, true);
    appendageStick1.NotifyNewData();
    frc3512::SubsystemBase::RunAllTeleopPeriodic();

    frc::sim::StepTiming(5_s);

    EXPECT_TRUE(climber.IsTelescopingOut());

    appendageStick1.SetRawButton(2, true);
    appendageStick1.NotifyNewData();
    frc3512::SubsystemBase::RunAllTeleopPeriodic();

    frc::sim::StepTiming(5_s);

    EXPECT_FALSE(climber.IsTelescopingOut());
}
