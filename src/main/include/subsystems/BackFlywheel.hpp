// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/Timer.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/filter/LinearFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/voltage.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "controllers/FlywheelController.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

/**
 * The back flywheel subsystem.
 *
 * The flywheel uses a Kalman filter for state estimation.
 */
class BackFlywheel : public ControlledSubsystemBase<1, 1, 1> {
public:
    /**
     * Constructs a rear Flywheel.
     */
    BackFlywheel();

    BackFlywheel(const BackFlywheel&) = delete;
    BackFlywheel& operator=(const BackFlywheel&) = delete;

    /**
     * Sets whether the robot will adjust the rear flywheel's
     * radians while the robot is moving
     *
     * @param moveAndShoot Whether or not to move and shoot.
     */
    void SetMoveAndShoot(bool moveAndShoot);

    /**
     * Returns angular displacement of the rear flywheel
     *
     * @return angular displacement in radians
     */

    units::radian_t GetAngle();

    /**
     * Returns angular velocity of the rear flywheel.
     *
     * @return angular velocity in radians per second
     */
    units::radians_per_second_t GetAngularVelocity() const;

    /**
     * Sets the goal of the controller.
     *
     * @param velocity The goal to pass to the controller in radians per second.
     */
    void SetGoal(units::radians_per_second_t velocity);

    /**
     * Returns the current goal of the controller.
     */
    units::radians_per_second_t GetGoal() const;

    /**
     * Sets the rear flywheel goal to zero
     */
    void Stop();

    /**
     * Returns true if the rear flywheel has reached the goal angular velocity.
     */
    bool AtGoal() const;

    /**
     * Returns true if the rear flywheel has been set to a nonzero goal.
     */
    bool IsOn() const;

    /**
     * Returns true if rear flywheel is spinning and it has reached the goal
     * angular velocity.
     */
    bool IsReady();

    /**
     * Resets sensors and the controller.
     */
    void Reset();

    /**
     * Returns current drawn in simulation.
     */
    units::ampere_t GetCurrentDraw() const;

    void DisabledInit() override { Disable(); }

    void AutonomousInit() override { Enable(); }

    void TeleopInit() override { Enable(); }

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void ControllerPeriodic() override;

private:
    rev::CANSparkMax m_backGrbx{HWConfig::Flywheel::kBackMotorID,
                                rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkMaxRelativeEncoder m_backEncoder{m_backGrbx.GetEncoder()};

    frc::LinearSystem<1, 1, 1> m_plant{FlywheelController::GetBackPlant()};
    frc::KalmanFilter<1, 1, 1> m_observer{
        m_plant,
        {200.0},
        {FlywheelController::kDpP / Constants::kControllerPeriod.value()},
        Constants::kControllerPeriod};

    FlywheelController m_controller{FlywheelPose::kBack};

    Eigen::Matrix<double, 1, 1> m_u = Eigen::Matrix<double, 1, 1>::Zero();

    bool m_moveAndShoot = true;

    units::radian_t m_angle;
    units::radian_t m_lastAngle;
    units::second_t m_time = frc::Timer::GetFPGATimestamp();
    units::second_t m_lastTime = m_time - Constants::kControllerPeriod;

    // Filters out encoder quantization noise
    units::radians_per_second_t m_angularVelocity;
    frc::LinearFilter<units::radians_per_second_t> m_velocityFilter =
        frc::LinearFilter<units::radians_per_second_t>::MovingAverage(4);

    // Used in test mode for manually setting flywheel goal. This is helpful for
    // measuring flywheel lookup table values.
    double m_testThrottle = 0.0;

    nt::NetworkTableEntry m_getGoalEntry =
        NetworkTableUtil::MakeDoubleEntry("Diagnostics/Back Flywheel/Get Goal");
    nt::NetworkTableEntry m_encoderEntry =
        NetworkTableUtil::MakeDoubleEntry("Diagnostics/Back Flywheel/Encoder");
    nt::NetworkTableEntry m_isReadyEntry =
        NetworkTableUtil::MakeBoolEntry("Diagnostics/Back Flywheel/Is Ready");
    nt::NetworkTableEntry m_atGoalEntry =
        NetworkTableUtil::MakeBoolEntry("Diagnostics/Back Flywheel/At Goal");
    nt::NetworkTableEntry m_manualRefEntry = NetworkTableUtil::MakeDoubleEntry(
        "Diagnostics/Back Flywheel/Manual Ref");

    /**
     * Sets the voltage of the flywheel motor.
     *
     * @param voltage The capped voltage to be set
     */
    void SetVoltage(units::volt_t voltage);

    /**
     * Computes an angular velocity reference from the provided joystick
     * throttle value.
     *
     * @param throttle Throttle value from -1 (forward) to 1 (back).
     */
    static units::radians_per_second_t ThrottleToReference(double throttle);
};

}  // namespace frc3512
