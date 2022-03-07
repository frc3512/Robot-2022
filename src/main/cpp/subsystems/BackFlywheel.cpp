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

    m_backGrbx.SetInverted(false);

    SetCANSparkMaxBusUsage(m_backGrbx, Usage::kAll);

    Reset();
    SetGoal(0_rad_per_s);
}

void BackFlywheel::SetMoveAndShoot(bool moveAndShoot) {
    m_moveAndShoot = moveAndShoot;
}

units::radian_t BackFlywheel::GetAngle() {
    return units::radian_t{m_backEncoder.GetPosition()};
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

}

void BackFlywheel::RobotPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    if (frc::DriverStation::IsTest()) {
        m_testThrottle = appendageStick1.GetThrottle();
        auto manualRef = ThrottleToReference(m_testThrottle);
        SetGoal(manualRef);
        m_manualRefEntry.SetDouble(manualRef.value());
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

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    m_lastAngle = m_angle;
    m_lastTime = m_time;
}

void BackFlywheel::SetVoltage(units::volt_t voltage) {
    m_backGrbx.SetVoltage(voltage);
}

units::radians_per_second_t BackFlywheel::ThrottleToReference(double throttle) {
    // 1. Remap input from [1..-1] to [0..1]
    auto remap = (1.0 - throttle) / 2.0;
    // 2. Rescale that to [400.0...800.0]
    constexpr auto kLow = 100_rad_per_s;
    constexpr auto kHigh = 800_rad_per_s;
    auto rescale = kLow + (kHigh - kLow) * remap;
    // 3. Round to the nearest radian per second
    return units::math::round(rescale);
}
