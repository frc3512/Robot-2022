// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"
#include "controllers/FlywheelConstants.hpp"

namespace frc3512 {

/**
 * The front flywheel controller.
 *
 * The flywheel uses an LQR for feedback control and a plant inversion
 * feedforward to maintain steady-state velocity.
 */
class FlywheelController : public ControllerBase<1, 1, 1> {
public:
    /// Gear ratio from encoder to both flywheels.
    static constexpr double kGearRatio = 1.0 / 1.0;

    /// Angle per encoder pulse.
    static constexpr double kDpP =
        (wpi::numbers::pi * 2.0) * kGearRatio / 2048.0;

    /**
     * Constructs a flywheel controller
     */
    explicit FlywheelController(FlywheelPose pose);

    /**
     * Move constructor.
     */
    FlywheelController(FlywheelController&&) = default;

    /**
     * Move assignment operator.
     */
    FlywheelController& operator=(FlywheelController&&) = default;

    /**
     * States of the front flywheel system.
     */
    class State {
    public:
        /// Front Flywheel angular velocity.
        static constexpr int kAngularVelocity = 0;
    };

    /**
     * Inputs of the front flywheel system.
     */
    class Input {
    public:
        /// Motor voltage.
        static constexpr int kVoltage = 0;
    };

    /**
     * Outputs of the flywheel system.
     */
    class Output {
    public:
        /// Flywheel angular velocity.
        static constexpr int kAngularVelocity = 0;
    };

    /**
     * Sets the goal.
     *
     * @param goal Angular velocity in radians per second
     */
    void SetGoal(units::radians_per_second_t goal);

    /**
     * Returns the goal.
     */
    units::radians_per_second_t GetGoal() const;

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Resets any internal state.
     */
    void Reset();

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    Eigen::Matrix<double, 1, 1> Calculate(
        const Eigen::Matrix<double, 1, 1>& x) override;

    /**
     * Returns the front flywheel plant.
     */
    static frc::LinearSystem<1, 1, 1> GetFrontPlant();

    /**
     * Returns the back flywheel plant.
     */
    static frc::LinearSystem<1, 1, 1> GetBackPlant();

private:
    static constexpr auto kAngularVelocityShotThreshold = 18_rad_per_s;
    static constexpr auto kAngularVelocityRecoveryThreshold = 18_rad_per_s;

    FlywheelPose m_pose;

    frc::LinearSystem<1, 1, 1> m_frontPlant{GetFrontPlant()};
    frc::LinearQuadraticRegulator<1, 1> m_frontLQR{
        m_frontPlant, {25.0}, {12.0}, Constants::kControllerPeriod};
    frc::LinearPlantInversionFeedforward<1, 1> m_frontFF{
        m_frontPlant, Constants::kControllerPeriod};

    frc::LinearSystem<1, 1, 1> m_backPlant{GetBackPlant()};
    frc::LinearQuadraticRegulator<1, 1> m_backLQR{
        m_backPlant, {25.0}, {12.0}, Constants::kControllerPeriod};
    frc::LinearPlantInversionFeedforward<1, 1> m_backFF{
        m_backPlant, Constants::kControllerPeriod};

    bool m_atGoal = false;

    /**
     * Update "at goal" flag based on next reference and current state estimate.
     *
     * This function applies hysteresis so AtGoal() doesn't chatter between true
     * and false.
     *
     * @param error The angular velocity error.
     */
    void UpdateAtGoal(units::radians_per_second_t error);
};
}  // namespace frc3512
