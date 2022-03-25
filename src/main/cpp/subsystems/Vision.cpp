// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <photonlib/PhotonUtils.h>

#include "HWConfig.hpp"
#include "TargetModel.hpp"

using namespace frc3512;

void Vision::TurnLEDOn() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOn); }

void Vision::TurnLEDOff() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOff); }

bool Vision::IsLEDOn() const {
    return m_rpiCam.GetLEDMode() == photonlib::LEDMode::kOn;
}

void Vision::SubscribeToVisionData(
    wpi::static_circular_buffer<GlobalMeasurements, 8>& queue) {
    m_subsystemQueues.push_back(&queue);
}

void Vision::UnsubscribeFromVisionData(
    wpi::static_circular_buffer<GlobalMeasurements, 8>& queue) {
    m_subsystemQueues.erase(
        std::remove(m_subsystemQueues.begin(), m_subsystemQueues.end(), &queue),
        m_subsystemQueues.end());
}

void Vision::UpdateVisionMeasurementsSim(const frc::Pose2d& drivetrainPose) {
    m_simVision.ProcessFrame(drivetrainPose);
}

bool Vision::HaveTargets() const { return m_haveTargets; }

void Vision::SwitchDriverMode(bool driveSwitch) {
    m_rpiCam.SetDriverMode(driveSwitch);
}

int Vision::GetPipelineIndex() { return m_rpiCam.GetPipelineIndex(); }

void Vision::SetPipeline(int pipeline) { m_rpiCam.SetPipelineIndex(pipeline); }

void Vision::RobotPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    const auto& result = m_rpiCam.GetLatestResult();

    if (result.GetTargets().size() == 0 || frc::DriverStation::IsDisabled()) {
        m_haveTargets = false;
        return;
    }

    m_haveTargets = true;

    auto timestamp = frc::Timer::GetFPGATimestamp() - result.GetLatency();

    m_timestampEntry.SetDouble(units::time::second_t{timestamp}.value());

    photonlib::PhotonTrackedTarget target = result.GetBestTarget();

    if (frc::DriverStation::IsTeleop()) {
        if (appendageStick1.GetRawButtonPressed(4)) {
            TurnLEDOn();
        } else if (appendageStick1.GetRawButtonPressed(6)) {
            TurnLEDOff();
        }
    }

    // Converts solvePnP() data from the NetworkTables to a global drivetrain
    // pose measurement
    auto cameraInTarget = target.GetCameraRelativePose();
    auto cameraInGlobal =
        TargetModel::kTargetPoseInGlobal.TransformBy(cameraInTarget);
    std::array<double, 3> pose{cameraInGlobal.X().value(),
                               cameraInGlobal.Y().value(),
                               cameraInGlobal.Rotation().Radians().value()};
    m_poseEntry.SetDoubleArray(pose);

    m_pitch = units::degree_t{target.GetPitch()};
    m_yaw = units::degree_t{target.GetYaw()};

    m_yawEntry.SetDouble(units::radian_t{m_yaw}.value());

    m_range = photonlib::PhotonUtils::CalculateDistanceToTarget(
        kCameraHeight, TargetModel::kCenter.Z(), kCameraPitch,
        units::degree_t{m_pitch});

    m_rangeEntry.SetDouble(units::meter_t{m_range}.value());

    for (auto& queue : m_subsystemQueues) {
        queue->push_back({m_yaw, m_pitch, m_range});
    }
}
