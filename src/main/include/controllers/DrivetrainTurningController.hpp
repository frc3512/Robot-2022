// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <tuple>
#include <vector>

#include <frc/Timer.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/LinearSystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/curvature.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include "Constants.hpp"
#include "LerpTable.hpp"
#include "controllers/ControllerBase.hpp"
#include "controllers/DrivetrainConstants.hpp"

namespace frc3512 {
/**
 * The drivetrain turning controller.
 */
class DrivetrainTurningController : public ControllerBase<3, 2, 3> {
public:
    /**
     * States of the drivetrain system.
     */
    class State {
    public:
        /// Heading in global coordinate frame.
        static constexpr int kHeading = 0;

        /// Left encoder velocity.
        static constexpr int kLeftVelocity = 1;

        /// Right encoder velocity.
        static constexpr int kRightVelocity = 2;
    };

    /**
     * Inputs of the drivetrain system.
     */
    class Input {
    public:
        /// Left motor voltage.
        static constexpr int kLeftVoltage = 0;

        /// Right motor voltage.
        static constexpr int kRightVoltage = 1;
    };

    /**
     * Local outputs of the drivetrain system.
     */
    class LocalOutput {
    public:
        /// Heading in global coordinate frame.
        static constexpr int kHeading = 0;

        /// Left encoder position.
        static constexpr int kLeftPosition = 1;

        /// Right encoder position.
        static constexpr int kRightPosition = 2;

        /// Acceleration along X axis in chassis coordinate frame.
        static constexpr int kAccelerationX = 3;

        /// Acceleration along Y axis in chassis coordinate frame.
        static constexpr int kAccelerationY = 4;
    };

    /**
     * Constructs a drivetrain turning controller
     */
    DrivetrainTurningController();

    /**
     * Move constructor.
     */
    DrivetrainTurningController(DrivetrainTurningController&&) = default;

    /**
     * Move assignment operator.
     */
    DrivetrainTurningController& operator=(DrivetrainTurningController&&) =
        default;

    /**
     * Sets the end goal of the controller profile.
     *
     * @param angle           Goal angle.
     * @param angularVelocity Goal angular velocity.
     */
    void SetGoal(units::radian_t angle,
                 units::radians_per_second_t angularVelocity);

    /**
     * Sets the references.
     *
     * @param angle  Angle of the carriage in radians.
     * @param angularVelocity  Angular velocity of the carriage in radians per
     *                         second.
     */
    void SetReferences(units::radian_t angle,
                       units::radians_per_second_t angularVelocity);

    /**
     * Returns true if drivetrain controller has a new heading goal.
     */
    bool HaveHeadingGoal() const;

    /**
     *  Abort turn in place actions.
     */
    void AbortTurnInPlace();

    /**
     * Returns whether the drivetrain controller is at the goal heading.
     */
    bool AtHeading() const;

    /**
     * Returns the next output of the turn-in-place controller.
     *
     * @param x The current state x.
     */
    Eigen::Vector<double, 2> Calculate(
        const Eigen::Vector<double, 3>& x) override;

    /**
     * Returns the drivetrain's plant.
     */
    static frc::LinearSystem<2, 2, 2> GetPlant();

    /**
     * The drivetrain system dynamics for turning in place.
     */
    static frc::LinearSystem<3, 2, 3> TurningDynamics();

    /**
     * Resets the controller.
     */
    void Reset(units::radian_t heading);

private:
    static constexpr double kVelocityTolerance = 2.0;  // radians/second
    static constexpr double kAngleTolerance = 0.52;    // radians

    frc::TrapezoidProfile<units::radian>::State m_goal;
    frc::TrapezoidProfile<units::radian>::Constraints m_constraints{
        DrivetrainConstants::Radians::kMaxAngularV,
        DrivetrainConstants::Radians::kMaxAngularA};
    frc::TrapezoidProfile<units::radian>::State m_profiledReference;

    static const frc::LinearSystem<2, 2, 2> kPlant;

    frc::LinearQuadraticRegulator<3, 2> m_lqr{TurningDynamics(),
                                              {0.001, 4.0, 4.0},
                                              {12.0, 12.0},
                                              Constants::kControllerPeriod};
    frc::LinearPlantInversionFeedforward<3, 2> m_ff{
        TurningDynamics(), Constants::kControllerPeriod};

    bool m_hasNewGoal = false;
    bool m_atReferences = false;

    /**
     * Update "at references" flag based on next reference and current state
     * estimate.
     *
     * @param error The error vector.
     */
    void UpdateAtReferences(const Eigen::Vector<double, 3>& error);
};
}  // namespace frc3512
