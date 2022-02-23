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
                              {ControllerLabel{"Angular velocity", "rad/s"}}) {
    m_frontGrbx.SetSmartCurrentLimit(40);
    m_backGrbx.SetSmartCurrentLimit(40);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_frontGrbx.Set(0.0);
    m_backGrbx.Set(0.0);

    m_frontGrbx.SetInverted(false);
    m_backGrbx.SetInverted(false);

    m_getGoalEntry.SetDouble(0.0);
    m_encoderEntry.SetDouble(0.0);
    m_isReadyEntry.SetBoolean(false);

    Reset();
    Stop();
}

void Flywheel::SetMoveAndShoot(bool moveAndShoot) {
    m_moveAndShoot = moveAndShoot;
}

units::radian_t Flywheel::GetAngle() {
    return units::radian_t{m_encoder.GetPosition()};
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

void Flywheel::Stop() { SetGoal(0_rad_per_s); }

bool Flywheel::AtGoal() const { return m_controller.AtGoal(); }

bool Flywheel::IsOn() const { return GetGoal() > 0_rad_per_s; }

bool Flywheel::IsReady() { return IsOn() && AtGoal(); }

void Flywheel::Reset() {
    m_observer.Reset();
    m_controller.Reset();
    m_u = Eigen::Matrix<double, 1, 1>::Zero();
    m_angle = GetAngle();
    m_lastAngle = m_angle;
}

void Flywheel::RobotPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (frc::DriverStation::IsTest()) {
        m_testThrottle = appendageStick2.GetThrottle();
        auto manualRef = ThrottleToReference(m_testThrottle);
        fmt::print("Manual angular velocity: {}\n",
                   units::revolutions_per_minute_t{manualRef});
    }

    if (!IsReady() && appendageStick1.GetRawButtonPressed(1)) {
        SetGoal(kShootLow);
    } else if (IsOn() && !AtGoal() && appendageStick1.GetRawButtonPressed(1)) {
        Stop();
    }

    if (!IsReady() && appendageStick2.GetRawButtonPressed(1)) {
        SetGoal(kShootHigh);
    } else if (IsOn() && !AtGoal() && appendageStick2.GetRawButtonPressed(1)) {
        Stop();
    }

    m_frontGrbx.Set(appendageStick1.GetRawAxis(1));

    m_getGoalEntry.SetDouble(GetGoal().value());
    m_encoderEntry.SetDouble(m_encoder.GetPosition());
    m_isReadyEntry.SetBoolean(IsReady());
}

void Flywheel::ControllerPeriodic() {
    using Input = FlywheelController::Input;

    UpdateDt();

    m_observer.Predict(m_u, GetDt());

    m_angle = GetAngle();
    m_time = frc::Timer::GetFPGATimestamp();

    m_angularVelocity = units::radians_per_second_t{m_encoder.GetVelocity()};

    Eigen::Matrix<double, 1, 1> y;
    y << GetAngularVelocity().value();
    m_observer.Correct(m_controller.GetInputs(), y);
    m_u = m_controller.Calculate(m_observer.Xhat());
    SetVoltage(units::volt_t{m_u(Input::kVoltage)});

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        units::volt_t voltage{m_frontGrbx.Get() *
                              frc::RobotController::GetInputVoltage()};
        if (m_flywheelSim.GetAngularVelocity() > 0_rad_per_s) {
            voltage -= FlywheelController::kS;
        } else if (m_flywheelSim.GetAngularVelocity() < 0_rad_per_s) {
            voltage += FlywheelController::kS;
        }

        m_flywheelSim.SetInput(Eigen::Vector<double, 1>{voltage.value()});
        m_flywheelSim.Update(GetDt());
    }

    m_lastAngle = m_angle;
    m_lastTime = m_time;
}

void Flywheel::SetVoltage(units::volt_t voltage) {
    m_frontGrbx.SetVoltage(voltage);
    m_backGrbx.SetVoltage(voltage);
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
