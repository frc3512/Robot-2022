// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <Eigen/Core>
#include <frc/ADIS16470_IMU.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include <frc/controller/ImplicitModelFollower.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/estimator/AngleStatistics.h>
#include <frc/estimator/KalmanFilterLatencyCompensator.h>
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/filter/LinearFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/simulation/ADIS16470_IMUSim.h>
#include <frc/simulation/AnalogInputSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <networktables/NetworkTableEntry.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

/**
 * The drivetrain subsystem.
 *
 * The drivetrain uses an unscented Kalman filter for state estimation.
 */
class Drivetrain : public ControlledSubsystemBase<7, 2, 5> {
public:
    /// The drivetrain length.
    static constexpr units::meter_t kLength = 0.9398_m;

    /**
     * Distance from middle of robot to intake.
     */
    static constexpr units::meter_t kMiddleOfRobotToIntake = 0.656_m;

    Drivetrain();

    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Returns the drivetrain's reference pose.
     */
    frc::Pose2d GetReferencePose() const;

    /**
     * Returns the drivetrain's pose estimate.
     */
    frc::Pose2d GetPose() const;

    /**
     * Returns gyro's heading measurement in the global coordinate frame.
     */
    units::radian_t GetAngle() const;

    /**
     * Returns left encoder displacement.
     */
    units::meter_t GetLeftPosition() const;

    /**
     * Returns right encoder displacement.
     */
    units::meter_t GetRightPosition() const;

    /**
     * Returns left encoder velocity.
     */
    units::meters_per_second_t GetLeftVelocity() const;

    /**
     * Returns right encoder velocity.
     */
    units::meters_per_second_t GetRightVelocity() const;

    /**
     * Returns longitudinal acceleration from IMU.
     */
    units::meters_per_second_squared_t GetAccelerationX() const;

    /**
     * Returns lateral acceleration from IMU.
     */
    units::meters_per_second_squared_t GetAccelerationY() const;

    /**
     * Resets all sensors and controller.
     */
    void Reset(const frc::Pose2d& initialPose = frc::Pose2d());

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void CorrectWithGlobalOutputs(units::meter_t x, units::meter_t y,
                                  units::second_t timestamp);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     */
    void AddTrajectory(const frc::Pose2d& start,
                       const std::vector<frc::Translation2d>& interior,
                       const frc::Pose2d& end);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     * @param config   TrajectoryConfig for this trajectory. This can include
     *                 constraints on the trajectory dynamics. If adding custom
     *                 constraints, it is recommended to start with the config
     *                 returned by MakeTrajectoryConfig() so differential drive
     *                 dynamics constraints are included automatically.
     */
    void AddTrajectory(const frc::Pose2d& start,
                       const std::vector<frc::Translation2d>& interior,
                       const frc::Pose2d& end,
                       const frc::TrajectoryConfig& config);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     */
    void AddTrajectory(const std::vector<frc::Pose2d>& waypoints);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     * @param config    TrajectoryConfig for this trajectory. This can include
     *                  constraints on the trajectory dynamics. If adding custom
     *                  constraints, it is recommended to start with the config
     *                  returned by MakeTrajectoryConfig() so differential drive
     *                  dynamics constraints are included automatically.
     */
    void AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
                       const frc::TrajectoryConfig& config);

    /**
     * Sets a new heading goal for the drivetrain to achieve.
     *
     * @param heading The heading you want to the drivetrain to achieve (in
     * radians)
     */
    void SetHeadingGoal(const units::radian_t heading);

    /**
     * Returns whether or not a new heading goal is set.
     */
    bool HasHeadingGoal() const;

    /**
     * Aborts turning in place.
     */
    void AbortTurnInPlace();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint with the start and end velocities set to zero.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint and the specified start and end velocities.
     *
     * @param startVelocity The start velocity of the trajectory.
     * @param endVelocity   The end velocity of the trajectory.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig(
        units::meters_per_second_t startVelocity,
        units::meters_per_second_t endVelocity);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Returns whether the drivetrain is at the goal heading.
     */
    bool AtHeading() const;

    /**
     * Returns the current heading state. Used for setting points after
     * turn-in-places
     */
    units::radian_t GetHeading();

    /**
     * Returns the drivetrain state estimate.
     */
    const Eigen::Vector<double, 7>& GetStates() const;

    /**
     * Returns the drivetrain inputs.
     */
    const Eigen::Vector<double, 2>& GetInputs() const;

    /**
     * Returns current drawn in simulation.
     */
    units::ampere_t GetCurrentDraw() const;

    /**
     * Returns sim pose.
     */
    frc::Pose2d GetSimPose() const;

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void TestInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void ControllerPeriodic() override;

private:
    static constexpr double kTurningP = 1.527459;
    static constexpr double kTurningI = 0.0;
    static constexpr double kTurningD = 0.0;

    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    static const frc::LinearSystem<2, 2, 2> kPlant;

    rev::CANSparkMax m_leftLeader{HWConfig::Drivetrain::kLeftMotorLeaderID,
                                  rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollower{HWConfig::Drivetrain::kLeftMotorFollowerID,
                                    rev::CANSparkMax::MotorType::kBrushless};
    frc::MotorControllerGroup m_leftGrbx{m_leftLeader, m_leftFollower};

    rev::CANSparkMax m_rightLeader{HWConfig::Drivetrain::kRightMotorLeaderID,
                                   rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollower{
        HWConfig::Drivetrain::kRightMotorFollowerID,
        rev::CANSparkMax::MotorType::kBrushless};
    frc::MotorControllerGroup m_rightGrbx{m_rightLeader, m_rightFollower};

    frc::Encoder m_leftEncoder{HWConfig::Drivetrain::kLeftEncoderA,
                               HWConfig::Drivetrain::kLeftEncoderB};

    frc::Encoder m_rightEncoder{HWConfig::Drivetrain::kRightEncoderA,
                                HWConfig::Drivetrain::kRightEncoderB};

    frc::ADIS16470_IMU m_imu;
    units::radian_t m_headingOffset = 0_rad;

    frc::UnscentedKalmanFilter<7, 2, 5> m_observer{
        DrivetrainController::Dynamics,
        DrivetrainController::LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5},
        {0.0001, 0.005, 0.005, 7.0, 7.0},
        frc::AngleMean<7, 7>(2),
        frc::AngleMean<5, 7>(0),
        frc::AngleResidual<7>(2),
        frc::AngleResidual<5>(0),
        frc::AngleAdd<7>(2),
        Constants::kControllerPeriod};
    frc::KalmanFilterLatencyCompensator<7, 2, 5,
                                        frc::UnscentedKalmanFilter<7, 2, 5>>
        m_latencyComp;
    DrivetrainController m_controller;
    Eigen::Vector<double, 2> m_u = Eigen::Vector<double, 2>::Zero();

    frc::TrapezoidProfile<units::radian>::Constraints m_turningConstraints{
        1.25_rad_per_s, 0.75_rad_per_s_sq};
    frc::ProfiledPIDController<units::radian> m_turningPID{
        kTurningP, kTurningI, kTurningD, m_turningConstraints,
        Constants::kControllerPeriod};
    bool m_hasNewHeading = false;

    frc::LinearSystem<2, 2, 2> m_imfRef =
        frc::LinearSystemId::IdentifyDrivetrainSystem(
            DrivetrainController::kLinearV,
            DrivetrainController::kLinearA * 5.0,
            DrivetrainController::kAngularV,
            DrivetrainController::kAngularA * 2.0);
    frc::ImplicitModelFollower<2, 2> m_imf{kPlant, m_imfRef, 20_ms};

    // Simulation variables
    frc::sim::DifferentialDrivetrainSim m_drivetrainSim{
        DrivetrainController::GetPlant(), DrivetrainController::kWidth,
        frc::DCMotor::NEO(2), DrivetrainController::kDriveGearRatio,
        DrivetrainController::kWheelRadius};
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::sim::ADIS16470_IMUSim m_imuSim{m_imu};
    frc::Field2d m_field;

    nt::NetworkTableEntry m_headingGoalEntry = NetworkTableUtil::MakeBoolEntry(
        "/Diagnostics/Drivetrain/Outputs/Goal Heading Achieved");

    nt::NetworkTableEntry m_hasHeadingGoalEntry =
        NetworkTableUtil::MakeBoolEntry(
            "/Diagnostics/Drivetrain/Outputs/Has New Goal Heading");

    /**
     * Set drivetrain motors to brake mode, which the feedback controllers
     * expect.
     */
    void SetBrakeMode();

    /**
     * Set drivetrain motors to coast mode so the robot is easier to push when
     * it's disabled.
     */
    void SetCoastMode();
};

}  // namespace frc3512
