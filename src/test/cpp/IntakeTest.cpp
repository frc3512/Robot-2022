// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <frc/simulation/JoystickSim.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Intake.hpp"

class IntakeTest : public frc3512::SimulatorTest {};

TEST_F(IntakeTest, Deploy) {
    frc::sim::JoystickSim appendageStick1{
        frc3512::HWConfig::kAppendageStick1Port};

    frc3512::Intake intake;

    frc3512::SubsystemBase::RunAllTeleopPeriodic();
    frc::Notifier controllerPeriodic{[&] { intake.TeleopPeriodic(); }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    appendageStick1.SetRawButton(4, true);
    appendageStick1.NotifyNewData();

    frc::sim::StepTiming(2_s);

    EXPECT_TRUE(intake.IsDeployed());

    appendageStick1.SetRawButton(5, true);
    appendageStick1.NotifyNewData();

    frc3512::SubsystemBase::RunAllTeleopPeriodic();

    EXPECT_FALSE(intake.IsDeployed());
}

TEST_F(IntakeTest, Conveyor) {
    frc::sim::JoystickSim appendageStick1{
        frc3512::HWConfig::kAppendageStick1Port};

    frc3512::Intake intake;

    frc3512::SubsystemBase::RunAllTeleopInit();
    frc::Notifier controllerPeriodic{[&] { intake.TeleopPeriodic(); }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    appendageStick1.SetRawButton(6, true);
    appendageStick1.NotifyNewData();

    frc3512::SubsystemBase::RunAllTeleopPeriodic();

    EXPECT_TRUE(intake.IsConveyorRunning());

    appendageStick1.SetRawButton(6, false);
    appendageStick1.NotifyNewData();

    frc3512::SubsystemBase::RunAllTeleopPeriodic();

    EXPECT_FALSE(intake.IsConveyorRunning());
}
