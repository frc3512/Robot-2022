// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Flywheel.hpp"

#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/fmt/Eigen.h>
#include <frc/fmt/Units.h>
#include <units/math.h>
#include <wpi/numbers>


using namespace frc3512;

Flywheel::Flywheel()
    : ControlledSubsystemBase("Flywheel",
                              {ControllerLabel{"Angular velocity", "rad/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Angular velocity", "rad/s"}})
    {
    m_leftGrbx.SetSmartCurrentLimit(40);
    m_rightGrbx.SetSmartCurrentLimit(40);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_encoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_encoder.SetSamplesToAverage(5);
    m_leftGrbx.SetInverted(false);
    m_rightGrbx.SetInverted(false);

    Reset();
    SetGoal(0_rad_per_s);
}

void Flywheel::SetMoveAndShoot(bool moveAndShoot) {
    m_moveAndShoot = moveAndShoot;
}

units::radian_t Flywheel::GetAngle() {
    return units::radian_t{m_encoder.GetDistance()};
}

units::radians_per_second_t Flywheel::GetAngularVelocity() const {
    return m_angularVelocity;
}

void Flywheel::SetGoal(units::radians_per_second_t velocity) {
    m_controller.SetGoal(velocity);
}

units::radians_per_second_t Flywheel::GetGoal() const {
    return m_controller.GetGoal();
}

bool Flywheel::AtGoal() const { return m_controller.AtGoal(); }

void Flywheel::SetHoodPose(HootPose pose)
{
    m_hoodPose = pose;
}

bool Flywheel::IsOn() const { return GetGoal() > 0_rad_per_s; }

bool Flywheel::IsReady() { return GetGoal() > 0_rad_per_s && AtGoal(); }

void Flywheel::Reset() {
    m_observer.Reset();
    m_controller.Reset();
    m_u = Eigen::Matrix<double, 1, 1>::Zero();
    m_encoder.Reset();
    m_angle = GetAngle();
    m_lastAngle = m_angle;
}

void Flywheel::RobotPeriodic() {
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (frc::DriverStation::GetInstance().IsTest()) {

        m_testThrottle = appendageStick2.GetThrottle();
        auto manualRef = ThrottleToReference(m_testThrottle);
        fmt::print("Manual angular velocity: {}\n",
                   units::revolutions_per_minute_t{manualRef});
    }

    if (appendageStick2.GetRawButtonPressed(3))
    {
        SetHoodPose(HoodPose::kHigh);
    } else if (appendageStick2.GetRawButtonPressed(4))
    {
        SetHoodPose(HoodPose::kLow);
    }
    if (appendageStick2.GetRawButtonPressed(1))
    {
        Shoot();
    }

    if (m_highGoalSwitch.Get() || m_lowGoalSwitch.Get())
    {
        m_hoodMotor.Set();
    }

    switch (m_hoodPose)
    {
        case HoodPose::kLow:
            if (!m_lowGoalSwitch.Get())
            {
                m_hoodMotor.Set(-0.75);
            } 
            break;
        case HoodPose::kHigh:
            if (!m_highGoalSwitch.Get())
            {
                m_hoodMotor.Set(0.75);
            }
            break;
        default:
            m_hoodMotor.Set(0.0);
            break;
    }
}

void Flywheel::ControllerPeriodic() {
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

    Eigen::Matrix<double, 1, 1> y;
    y << GetAngularVelocity().value();
    m_observer.Correct(m_controller.GetInputs(), y);
    m_u = m_controller.Calculate(m_observer.Xhat());
    SetVoltage(units::volt_t{m_u(Input::kVoltage)});

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);


    m_lastAngle = m_angle;
    m_lastTime = m_time;
}

void Flywheel::SetVoltage(units::volt_t voltage) {
    m_leftGrbx.SetVoltage(voltage);
    m_rightGrbx.SetVoltage(-voltage);
}

units::radians_per_second_t Flywheel::ThrottleToReference(double throttle) {
    // 1. Remap input from [1..-1] to [0..1]
    auto remap = (1.0 - throttle) / 2.0;
    // 2. Rescale that to [400.0...800.0]
    constexpr auto kLow = 400_rad_per_s;
    constexpr auto kHigh = 800_rad_per_s;
    auto rescale = kLow + (kHigh - kLow) * remap;
    // 3. Round to the nearest radian per second
    return units::math::round(rescale);
}

void Flywheel::Shoot()
{
    if (!m_lowGoalSwitch.Get() && !m_highGoalSwitch.Get())
    {
        m_hoodPose = HoodPose::kHigh;
    }

    SetGoal(kShootSpeed);
}
