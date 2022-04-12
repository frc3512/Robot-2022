// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/FrontFlywheel.hpp"

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

FrontFlywheel::FrontFlywheel()
    : ControlledSubsystemBase("Front Flywheel",
                              {ControllerLabel{"Angular velocity", "rad/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Angular velocity", "rad/s"}}) {
    m_frontGrbx.SetSmartCurrentLimit(40);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_frontGrbx.Set(0.0);

    m_frontGrbx.SetInverted(true);

    m_frontEncoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_frontEncoder.SetSamplesToAverage(5);
    m_frontEncoder.SetReverseDirection(true);

    SetCANSparkMaxBusUsage(m_frontGrbx, Usage::kMinimal);

    // currently flywheel goes to constants speed, but might need to increase
    // for farther shots. using LerpTable for furture proofing.
    m_table.Insert(12_in, 359_rad_per_s);  // hood down.
    m_table.Insert(24_in, 359_rad_per_s);  // hood up.
    m_table.Insert(48_in, 359_rad_per_s);
    m_table.Insert(72_in, 359_rad_per_s);
    m_table.Insert(96_in, 359_rad_per_s);
    m_table.Insert(108_in, 414.89_rad_per_s);

    Reset();
    SetGoal(0_rad_per_s);
}

units::radian_t FrontFlywheel::GetAngle() {
    return units::radian_t{m_frontEncoder.GetDistance()};
}

units::radians_per_second_t FrontFlywheel::GetAngularVelocity() const {
    return m_angularVelocity;
}

void FrontFlywheel::SetGoal(units::radians_per_second_t velocity) {
    m_controller.SetGoal(velocity);
}

units::radians_per_second_t FrontFlywheel::GetGoal() const {
    return m_controller.GetGoal();
}

units::radians_per_second_t FrontFlywheel::GetGoalFromRange() {
    return m_table[m_range];
}

void FrontFlywheel::Stop() { SetGoal(0_rad_per_s); }

bool FrontFlywheel::AtGoal() const { return m_controller.AtGoal(); }

bool FrontFlywheel::IsOn() const { return GetGoal() > 0_rad_per_s; }

bool FrontFlywheel::IsReady() { return IsOn() && AtGoal(); }

void FrontFlywheel::Reset() {
    m_observer.Reset();
    m_controller.Reset();
    m_u = Eigen::Matrix<double, 1, 1>::Zero();
    m_angle = GetAngle();
    m_lastAngle = m_angle;
}

void FrontFlywheel::TeleopPeriodic() {
    double percent = GetGoal().value() /
                     FrontFlywheelConstants::kMaxAngularVelocity.value() *
                     100.0;
    m_percentageEntry.SetDouble(percent);
}

void FrontFlywheel::RobotPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (frc::DriverStation::IsTest()) {
        m_testThrottle = appendageStick2.GetThrottle();
        auto manualRef = ThrottleToReference(m_testThrottle);
        SetGoal(manualRef);
        double percent = (manualRef.value() /
                          FrontFlywheelConstants::kMaxAngularVelocity.value()) *
                         100.0;
        m_percentageEntry.SetDouble(percent);
    }
}

void FrontFlywheel::ControllerPeriodic() {
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
        units::volt_t voltage{m_frontGrbx.Get() *
                              frc::RobotController::GetInputVoltage()};
        if (m_flywheelSim.GetAngularVelocity() > 0_rad_per_s) {
            voltage -= FrontFlywheelConstants::kS;
        } else if (m_flywheelSim.GetAngularVelocity() < 0_rad_per_s) {
            voltage += FrontFlywheelConstants::kS;
        }

        m_flywheelSim.SetInput(Eigen::Vector<double, 1>{voltage.value()});
        m_flywheelSim.Update(GetDt());
        m_encoderSim.SetDistance(m_flywheelSim.GetAngle().value());
    }

    m_lastAngle = m_angle;
    m_lastTime = m_time;
}

units::ampere_t FrontFlywheel::GetCurrentDraw() const {
    return m_flywheelSim.GetCurrentDraw();
}

void FrontFlywheel::SetSimAngularVelocity(
    units::radians_per_second_t velocity) {
    m_flywheelSim.SetState(Eigen::Vector<double, 2>{
        m_flywheelSim.GetAngle().value(), velocity.value()});
}

void FrontFlywheel::SetVoltage(units::volt_t voltage) {
    m_frontGrbx.SetVoltage(voltage);
}

units::radians_per_second_t FrontFlywheel::ThrottleToReference(
    double throttle) {
    // 1. Remap input from [1..-1] to [0..1]
    auto remap = (1.0 - throttle) / 2.0;
    // 2. Rescale that to [400.0...800.0]
    constexpr auto kLow = 100_rad_per_s;
    constexpr auto kHigh = 1500_rad_per_s;
    auto rescale = kLow + (kHigh - kLow) * remap;
    // 3. Round to the nearest radian per second
    return units::math::round(rescale);
}
