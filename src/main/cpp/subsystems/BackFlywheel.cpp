// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/BackFlywheel.hpp"

#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/fmt/Eigen.h>
#include <frc/fmt/Units.h>
#include <units/math.h>
#include <wpi/numbers>

#include "CANSparkMaxUtil.hpp"

using namespace frc3512;

BackFlywheel::BackFlywheel()
    : ControlledSubsystemBase("Back Flywheel",
                              {ControllerLabel{"Angular velocity", "rad/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Angular velocity", "rad/s"}}) {
    m_backGrbx.SetSmartCurrentLimit(40);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_backGrbx.Set(0.0);

    m_backGrbx.SetInverted(true);

    m_backEncoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_backEncoder.SetSamplesToAverage(5);

    SetCANSparkMaxBusUsage(m_backGrbx, Usage::kMinimal);

    m_table.Insert(12_in, 100_rad_per_s * kSpeedOffset);  // hood down.
    m_table.Insert(24_in, 100_rad_per_s * kSpeedOffset);  // hood up.
    m_table.Insert(48_in, 166_rad_per_s * kSpeedOffset);
    m_table.Insert(72_in, 221_rad_per_s * kSpeedOffset);
    m_table.Insert(96_in, 271_rad_per_s * kSpeedOffset);
    m_table.Insert(108_in, 271_rad_per_s * kSpeedOffset);

    Reset();
    SetGoal(0_rad_per_s);
}

void BackFlywheel::DeployHood() { m_solenoid.Set(false); }

void BackFlywheel::StowHood() { m_solenoid.Set(true); }

bool BackFlywheel::IsHoodDeployed() { return !m_solenoid.Get(); }

units::radian_t BackFlywheel::GetAngle() {
    return units::radian_t{m_backEncoder.GetDistance()};
}

units::radians_per_second_t BackFlywheel::GetAngularVelocity() const {
    return m_angularVelocity;
}

void BackFlywheel::SetGoal(units::radians_per_second_t velocity) {
    m_controller.SetGoal(velocity);
}

units::radians_per_second_t BackFlywheel::GetGoal() const {
    return m_controller.GetGoal();
}

units::radians_per_second_t BackFlywheel::GetGoalFromRange() {
    return m_table[m_range];
}

void BackFlywheel::Stop() { SetGoal(0_rad_per_s); }

bool BackFlywheel::AtGoal() const { return m_controller.AtGoal(); }

bool BackFlywheel::IsOn() const { return GetGoal() > 0_rad_per_s; }

bool BackFlywheel::IsReady() { return IsOn() && AtGoal(); }

void BackFlywheel::Reset() {
    m_observer.Reset();
    m_controller.Reset();
    m_u = Eigen::Matrix<double, 1, 1>::Zero();
    m_angle = GetAngle();
    m_lastAngle = m_angle;
}

void BackFlywheel::TeleopPeriodic() {
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    if (driveStick2.GetRawButtonPressed(4)) {
        if (IsHoodDeployed()) {
            StowHood();
        } else {
            DeployHood();
        }
    }

    double percent = GetGoal().value() /
                     BackFlywheelConstants::kMaxAngularVelocity.value() * 100.0;
    m_percentageEntry.SetDouble(percent);
}

void BackFlywheel::RobotPeriodic() {
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    if (frc::DriverStation::IsTest()) {
        m_testThrottle = appendageStick1.GetThrottle();
        auto manualRef = ThrottleToReference(m_testThrottle);
        SetGoal(manualRef);
        double percent = (manualRef.value() /
                          BackFlywheelConstants::kMaxAngularVelocity.value()) *
                         100.0;
        m_percentageEntry.SetDouble(percent);

        if (driveStick2.GetRawButtonPressed(4)) {
            if (IsHoodDeployed()) {
                StowHood();
            } else {
                DeployHood();
            }
        }
    }
}

void BackFlywheel::ControllerPeriodic() {
    using Input = FlywheelController::Input;

    UpdateDt();

    m_observer.Predict(m_u, GetDt());

    m_angle = GetAngle();
    m_time = frc::Timer::GetFPGATimestamp();

    // WPILib uses the time between pulses in GetRate() to calculate velocity,
    // but this is very noisy for high-resolution encoders. Instead, we
    // calculate a velocity from the change in angle over change in time, which
    // is more precise.
    m_angularVelocity = m_velocityFilter.Calculate((m_angle - m_lastAngle) /
                                                   (m_time - m_lastTime));

    Eigen::Matrix<double, 1, 1> y{GetAngularVelocity().value()};
    m_observer.Correct(m_controller.GetInputs(), y);
    m_u = m_controller.Calculate(m_observer.Xhat());
    SetVoltage(units::volt_t{m_u(Input::kVoltage)});

    while (visionQueue.size() > 0) {
        auto measurement = visionQueue.pop_front();
        m_range = measurement.range;
    }

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        units::volt_t voltage{m_backGrbx.Get() *
                              frc::RobotController::GetInputVoltage()};
        if (m_flywheelSim.GetAngularVelocity() > 0_rad_per_s) {
            voltage -= BackFlywheelConstants::kS;
        } else if (m_flywheelSim.GetAngularVelocity() < 0_rad_per_s) {
            voltage += BackFlywheelConstants::kS;
        }

        m_flywheelSim.SetInput(Eigen::Vector<double, 1>{voltage.value()});
        m_flywheelSim.Update(GetDt());
        m_encoderSim.SetDistance(m_flywheelSim.GetAngle().value());
    }

    m_lastAngle = m_angle;
    m_lastTime = m_time;
}

units::ampere_t BackFlywheel::GetCurrentDraw() const {
    return m_flywheelSim.GetCurrentDraw();
}

void BackFlywheel::SetSimAngularVelocity(units::radians_per_second_t velocity) {
    m_flywheelSim.SetState(Eigen::Vector<double, 2>{
        m_flywheelSim.GetAngle().value(), velocity.value()});
}

void BackFlywheel::SetVoltage(units::volt_t voltage) {
    m_backGrbx.SetVoltage(voltage);
}

units::radians_per_second_t BackFlywheel::ThrottleToReference(double throttle) {
    // 1. Remap input from [1..-1] to [0..1]
    auto remap = (1.0 - throttle) / 2.0;
    // 2. Rescale that to [400.0...800.0]
    constexpr auto kLow = 100_rad_per_s;
    constexpr auto kHigh = 1500_rad_per_s;
    auto rescale = kLow + (kHigh - kLow) * remap;
    // 3. Round to the nearest radian per second
    return units::math::round(rescale);
}
