// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <algorithm>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/fmt/Eigen.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "CANSparkMaxUtil.hpp"

using namespace frc3512;

const Eigen::Matrix<double, 2, 2> Drivetrain::kGlobalR =
    frc::MakeCovMatrix(0.2, 0.2);

frc::LinearSystem<2, 2, 2> Drivetrain::kPlant{DrivetrainController::GetPlant()};

Drivetrain::Drivetrain()
    : ControlledSubsystemBase(
          "Drivetrain",
          {ControllerLabel{"X", "m"}, ControllerLabel{"Y", "m"},
           ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left velocity", "m/s"},
           ControllerLabel{"Right velocity", "m/s"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"}},
          {ControllerLabel{"Left voltage", "V"},
           ControllerLabel{"Right voltage", "V"}},
          {ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"},
           ControllerLabel{"Longitudinal Acceleration", "m/s^2"},
           ControllerLabel{"Lateral Acceleration", "m/s^2"}},
          true) {
    SetCANSparkMaxBusUsage(m_leftLeader, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_leftFollower, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightLeader, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightFollower, Usage::kMinimal);

    m_leftLeader.SetSmartCurrentLimit(60);
    m_leftFollower.SetSmartCurrentLimit(60);
    m_rightLeader.SetSmartCurrentLimit(60);
    m_rightFollower.SetSmartCurrentLimit(60);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftGrbx.SetInverted(true);

    m_leftEncoder.SetSamplesToAverage(10);
    m_rightEncoder.SetSamplesToAverage(10);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);
    m_leftEncoder.SetDistancePerPulse(DrivetrainController::kDpP);
    m_rightEncoder.SetDistancePerPulse(DrivetrainController::kDpP);

    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    Reset(frc::Pose2d{0_m, 0_m, 0_rad});

    m_turningPID.EnableContinuousInput(units::radian_t{-wpi::numbers::pi},
                                       units::radian_t{wpi::numbers::pi});
    m_turningPID.SetTolerance(units::radian_t{0.25},
                              units::radians_per_second_t{2});

    frc::SmartDashboard::PutData(&m_field);
}

frc::Pose2d Drivetrain::GetReferencePose() const {
    const auto& x = m_controller.GetReferences();
    return frc::Pose2d{
        units::meter_t{x(DrivetrainController::State::kX)},
        units::meter_t{x(DrivetrainController::State::kY)},
        units::radian_t{x(DrivetrainController::State::kHeading)}};
}

frc::Pose2d Drivetrain::GetPose() const { return m_observer.GetPose(); }

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{m_imu.GetAngle()} + m_headingOffset;
}

units::meter_t Drivetrain::GetLeftPosition() const {
    return units::meter_t{m_leftEncoder.GetDistance()};
}

units::meter_t Drivetrain::GetRightPosition() const {
    return units::meter_t{m_rightEncoder.GetDistance()};
}

units::meters_per_second_t Drivetrain::GetLeftVelocity() const {
    return m_leftVelocity;
}

units::meters_per_second_t Drivetrain::GetRightVelocity() const {
    return m_rightVelocity;
}

units::meters_per_second_squared_t Drivetrain::GetAccelerationX() const {
    return -m_imu.GetAccelX();
}

units::meters_per_second_squared_t Drivetrain::GetAccelerationY() const {
    return -m_imu.GetAccelY();
}

void Drivetrain::Reset(const frc::Pose2d& initialPose) {
    using State = frc::sim::DifferentialDrivetrainSim::State;

    m_controller.Reset(initialPose);
    m_u = Eigen::Vector<double, 2>::Zero();
    m_turningPID.Reset(initialPose.Rotation().Radians());
    m_hasNewHeading = false;
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_imu.Reset();
    m_headingOffset = initialPose.Rotation().Radians();
    m_visionTimer.Reset();

    Eigen::Vector<double, 7> xHat;
    xHat(State::kX) = initialPose.X().value();
    xHat(State::kY) = initialPose.Y().value();
    xHat(State::kHeading) = initialPose.Rotation().Radians().value();
    xHat.block<4, 1>(3, 0).setZero();
    m_xHat = xHat;
    m_observer.ResetPosition(initialPose, GetAngle());
    Eigen::Vector<double, 2> filterXHat = Eigen::Vector<double, 2>::Zero();
    m_velocityObserver.SetXhat(filterXHat);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_drivetrainSim.SetState(xHat);
        m_field.SetRobotPose(initialPose);
    }
}

void Drivetrain::ControllerPeriodic() {
    using Input = DrivetrainController::Input;

    UpdateDt();

    m_velocityObserver.Predict(m_u, GetDt());

    m_leftPos = GetLeftPosition();
    m_rightPos = GetRightPosition();
    m_time = frc::Timer::GetFPGATimestamp();

    // WPILib uses the time between pulses in GetRate() to calculate velocity,
    // but this is very noisy for high-resolution encoders. Instead, we
    // calculate a velocity from the change in position over change in time,
    // which is more precise.
    auto rawLeftVelocity = (m_leftPos - m_lastLeftPos) / (m_time - m_lastTime);
    auto rawRightVelocity =
        (m_rightPos - m_lastRightPos) / (m_time - m_lastTime);

    m_leftVelocity = m_leftVelocityFilter.Calculate(rawLeftVelocity);
    m_rightVelocity = m_rightVelocityFilter.Calculate(rawRightVelocity);

    Eigen::Vector<double, 2> velPosY{GetLeftVelocity().value(),
                                     GetRightVelocity().value()};

    Eigen::Vector<double, 5> y{GetAngle().value(), GetLeftPosition().value(),
                               GetRightPosition().value(),
                               GetAccelerationX().value(),
                               GetAccelerationY().value()};
    m_observer.Update(GetAngle(), GetLeftPosition(), GetRightPosition());

    m_velocityObserver.Correct(m_controller.GetInputs(), velPosY);

    Eigen::Vector<double, 7> controllerState = GetStates();

    while (visionQueue.size() > 0) {
        auto measurement = visionQueue.pop_front();

        m_controller.SetVisionYaw(measurement.yaw);
        m_controller.SetVisionRange(measurement.range);
    }

    if (m_controller.HaveTrajectory()) {
        m_u = m_controller.Calculate(GetStates());

        if (!AtGoal()) {
            m_leftGrbx.SetVoltage(units::volt_t{m_u(Input::kLeftVoltage)});
            m_rightGrbx.SetVoltage(units::volt_t{m_u(Input::kRightVoltage)});
        } else {
            m_leftGrbx.SetVoltage(0_V);
            m_rightGrbx.SetVoltage(0_V);
        }
    } else if (HasHeadingGoal()) {
        // Update previous u stored in the controller. We don't care what the
        // return value is.
        static_cast<void>(m_controller.Calculate(GetStates()));

        if (!AtHeading()) {
            auto turningOutput = m_turningPID.Calculate(units::radian_t{
                controllerState(DrivetrainController::State::kHeading)});
            m_leftGrbx.SetVoltage(units::volt_t{-turningOutput} +
                                  m_turningFeedforward.Calculate(
                                      m_turningPID.GetGoal().velocity));
            m_rightGrbx.SetVoltage(units::volt_t{turningOutput} +
                                   m_turningFeedforward.Calculate(
                                       m_turningPID.GetGoal().velocity));
        } else {
            m_hasNewHeading = false;
            m_leftGrbx.SetVoltage(0_V);
            m_rightGrbx.SetVoltage(0_V);
        }
    } else {
        // Update previous u stored in the controller. We don't care what the
        // return value is.
        static_cast<void>(m_controller.Calculate(GetStates()));

        // Run observer predict with inputs from teleop
        m_u = Eigen::Vector<double, 2>{
            std::clamp(m_leftGrbx.Get(), -1.0, 1.0) *
                frc::RobotController::GetInputVoltage(),
            std::clamp(m_rightGrbx.Get(), -1.0, 1.0) *
                frc::RobotController::GetInputVoltage()};

        m_turningPID.Calculate(units::radian_t{
            controllerState(DrivetrainController::State::kHeading)});
    }

    if ((!(m_controller.GetVisionYaw() < 0.1_rad) ||
         !(m_controller.GetVisionYaw() > -0.1_rad)) &&
        m_aimWithVision && !m_visionTimer.HasElapsed(3_s)) {
        SetHeadingGoal(GetAngle() - m_controller.GetVisionYaw());
    } else {
        m_visionTimer.Stop();
        m_visionTimer.Reset();
        m_aimWithVision = false;
    }

    m_currHeadingEntry.SetDouble(m_turningPID.GetGoal().position.value());
    m_headingGoalValueEntry.SetDouble(m_turningPID.GetGoal().position.value());

    Log(m_controller.GetReferences(), GetStates(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        auto batteryVoltage = frc::RobotController::GetInputVoltage();
        m_drivetrainSim.SetInputs(
            units::volt_t{std::clamp(m_leftGrbx.Get(), -1.0, 1.0) *
                          batteryVoltage},
            units::volt_t{std::clamp(m_rightGrbx.Get(), -1.0, 1.0) *
                          batteryVoltage});

        m_drivetrainSim.Update(GetDt());

        m_leftEncoderSim.SetDistance(m_drivetrainSim.GetLeftPosition().value());
        m_leftEncoderSim.SetRate(m_drivetrainSim.GetLeftVelocity().value());
        m_rightEncoderSim.SetDistance(
            m_drivetrainSim.GetRightPosition().value());
        m_rightEncoderSim.SetRate(m_drivetrainSim.GetRightVelocity().value());
        m_imuSim.SetGyroAngleZ(
            units::degree_t{m_drivetrainSim.GetHeading().Radians()} -
            m_headingOffset);

        const auto& plant = DrivetrainController::GetPlant();
        Eigen::Vector<double, 2> x{m_drivetrainSim.GetLeftVelocity().value(),
                                   m_drivetrainSim.GetRightVelocity().value()};
        Eigen::Vector<double, 2> u{
            std::clamp(m_leftGrbx.Get(), -1.0, 1.0) * batteryVoltage,
            std::clamp(m_rightGrbx.Get(), -1.0, 1.0) * batteryVoltage};
        Eigen::Vector<double, 2> xdot = plant.A() * x + plant.B() * u;
        m_imuSim.SetAccelX(
            -units::meters_per_second_squared_t{(xdot(0) + xdot(1)) / 2.0});

        units::meters_per_second_t leftVelocity{x(0)};
        units::meters_per_second_t rightVelocity{x(1)};
        m_imuSim.SetAccelY(
            -((rightVelocity * rightVelocity) - (leftVelocity * leftVelocity)) /
            (2.0 * DrivetrainController::kWidth));

        m_field.SetRobotPose(m_drivetrainSim.GetPose());
        m_headingGoalEntry.SetBoolean(AtHeading());
        m_atGoalEntry.SetBoolean(AtGoal());
    }

    m_lastLeftPos = m_leftPos;
    m_lastRightPos = m_rightPos;
    m_lastTime = m_time;
}

void Drivetrain::RobotPeriodic() {}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end) {
    m_controller.AddTrajectory(start, interior, end);
}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(start, interior, end, config);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints) {
    m_controller.AddTrajectory(waypoints);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(waypoints, config);
}

void Drivetrain::AddTrajectory(const frc::Trajectory trajectory,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(trajectory, config);
}

void Drivetrain::SetHeadingGoal(const units::radian_t heading) {
    m_turningPID.SetGoal(frc::AngleModulus(heading));
    m_hasNewHeading = true;
}

bool Drivetrain::HasHeadingGoal() const { return m_hasNewHeading; }

void Drivetrain::AbortTurnInPlace() {
    Eigen::Vector<double, 7> controllerState = GetStates();
    m_turningPID.SetGoal(units::radian_t{
        controllerState(DrivetrainController::State::kHeading)});
    m_hasNewHeading = false;
}

frc::TrajectoryConfig Drivetrain::MakeTrajectoryConfig() {
    return DrivetrainController::MakeTrajectoryConfig();
}

frc::TrajectoryConfig Drivetrain::MakeTrajectoryConfig(
    units::meters_per_second_t startVelocity,
    units::meters_per_second_t endVelocity) {
    return DrivetrainController::MakeTrajectoryConfig(startVelocity,
                                                      endVelocity);
}

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

bool Drivetrain::AtHeading() { return m_turningPID.AtGoal(); }

units::radian_t Drivetrain::GetHeading() {
    Eigen::Vector<double, 7> controllerState = GetStates();
    return units::radian_t{
        controllerState(DrivetrainController::State::kHeading)};
}

const Eigen::Vector<double, 7>& Drivetrain::GetStates() {
    using VelocityState = DrivetrainController::VelocityFilterState;
    m_xHat = Eigen::Vector<double, 7>{
        GetPose().X().value(),
        GetPose().Y().value(),
        GetAngle().value(),
        m_velocityObserver.Xhat(VelocityState::kLeftVelocity),
        m_velocityObserver.Xhat(VelocityState::kRightVelocity),
        GetLeftPosition().value(),
        GetRightPosition().value()};
    return m_xHat;
}

const Eigen::Vector<double, 2>& Drivetrain::GetInputs() const {
    return m_controller.GetInputs();
}

units::ampere_t Drivetrain::GetCurrentDraw() const {
    return m_drivetrainSim.GetCurrentDraw();
}

frc::Pose2d Drivetrain::GetSimPose() const { return m_drivetrainSim.GetPose(); }

units::radian_t Drivetrain::GetVisionYaw() {
    return m_controller.GetVisionYaw();
}

void Drivetrain::AimWithVision() {
    m_visionTimer.Start();
    m_aimWithVision = true;
}

void Drivetrain::DisabledInit() {
    SetBrakeMode();
    Disable();
}

void Drivetrain::AutonomousInit() {
    SetBrakeMode();
    Enable();
}

void Drivetrain::TeleopInit() {
    SetBrakeMode();

    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    m_controller.AbortTrajectories();

    // If the robot was disabled while still turning in place in
    // autonomous, it will continue to do so in teleop. This aborts any
    // turning action so teleop driving can occur.
    AbortTurnInPlace();

    m_turningPID.SetTolerance(units::radian_t{0.5},
                              units::radians_per_second_t{2});

    Enable();
}

void Drivetrain::TestInit() {
    SetBrakeMode();

    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    m_controller.AbortTrajectories();

    // If the robot was disabled while still turning in place in
    // autonomous, it will continue to do so in teleop. This aborts any
    // turning action so teleop driving can occur.
    AbortTurnInPlace();

    Enable();
}

void Drivetrain::TeleopPeriodic() {
    using Input = DrivetrainController::Input;

    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    double y =
        frc::ApplyDeadband(-driveStick1.GetY(), Constants::kJoystickDeadband);
    double x =
        frc::ApplyDeadband(driveStick2.GetX(), Constants::kJoystickDeadband);

    if (driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }

    auto [left, right] = frc::DifferentialDrive::CurvatureDriveIK(
        y, x, driveStick2.GetRawButton(2));

    // Implicit model following
    // TODO: Velocities need filtering
    Eigen::Vector<double, 2> u =
        m_imf.Calculate(Eigen::Vector<double, 2>{GetLeftVelocity().value(),
                                                 GetRightVelocity().value()},
                        Eigen::Vector<double, 2>{left * 12.0, right * 12.0});

    m_leftGrbx.SetVoltage(units::volt_t{u(Input::kLeftVoltage)});
    m_rightGrbx.SetVoltage(units::volt_t{u(Input::kRightVoltage)});

    if (driveStick1.GetRawButtonPressed(3)) {
        SetHeadingGoal(GetAngle() + units::radian_t{wpi::numbers::pi});
    }

    m_headingGoalEntry.SetBoolean(AtHeading());
    m_hasHeadingGoalEntry.SetBoolean(HasHeadingGoal());

    m_yawControllerEntry.SetDouble(GetVisionYaw().value());
    m_rangeControllerEntry.SetDouble(m_controller.GetVisionRange().value());
}

void Drivetrain::TestPeriodic() {
    using Input = DrivetrainController::Input;

    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    double y =
        frc::ApplyDeadband(-driveStick1.GetY(), Constants::kJoystickDeadband);
    double x =
        frc::ApplyDeadband(driveStick2.GetX(), Constants::kJoystickDeadband);

    if (driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }
    auto [left, right] = frc::DifferentialDrive::CurvatureDriveIK(
        y, x, driveStick2.GetRawButton(2));

    // Implicit model following
    // TODO: Velocities need filtering
    Eigen::Vector<double, 2> u =
        m_imf.Calculate(Eigen::Vector<double, 2>{GetLeftVelocity().value(),
                                                 GetRightVelocity().value()},
                        Eigen::Vector<double, 2>{left * 12.0, right * 12.0});

    if (!HasHeadingGoal()) {
        m_leftGrbx.SetVoltage(units::volt_t{u(Input::kLeftVoltage)});
        m_rightGrbx.SetVoltage(units::volt_t{u(Input::kRightVoltage)});
    }

    m_headingGoalEntry.SetBoolean(AtHeading());
    m_hasHeadingGoalEntry.SetBoolean(HasHeadingGoal());
}

void Drivetrain::SetBrakeMode() {
    m_leftLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Drivetrain::SetCoastMode() {
    m_leftLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightLeader.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}
