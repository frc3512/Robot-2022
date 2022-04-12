// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimVisionSystem.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <wpi/static_circular_buffer.h>

#include "Constants.hpp"
#include "NetworkTableUtil.hpp"
#include "TargetModel.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * Vision subsystem
 */
class Vision : public SubsystemBase {
public:
    /// Vision Targeting Camera name in Networktables
    static constexpr char kVisionCamName[] = "mmal_service_16.1";

    /// Driver Camera name in Networktables
    static constexpr char kDriverCamName[] = "USB_CAM";

    /// Pipeline index for upper hub vision tracking
    static constexpr int kTargetPipeline = 0;

    /// Pipeline index for blue cargo vision tracking
    static constexpr int kBlueBallPipeline = 1;

    /// Pipeline index for red cargo vision tracking
    static constexpr int kRedBallPipeline = 2;

    /// Target Height
    static constexpr units::meter_t kTargetHeight = 2.6_m;

    /// Camera Height
    static constexpr units::meter_t kCameraHeight = 0.76835_m;

    /// Camera Pitch
    static constexpr units::degree_t kCameraPitch = 44.6_deg;

    /// Pi camera V1 diagonal field of view
    static constexpr units::degree_t kCameraDiagonalFOV = 74.8_deg;

    /// The yaw offset of the vision camera.
    static constexpr units::degree_t kYawOffset = 3.5_deg;

    /**
     * Container for global measurements.
     *
     * These are sent to the drivetrian and flywheel subsystems via a
     * producer-consumer queue.
     */
    struct GlobalMeasurements {
        /// Yaw reported by photonvision
        units::radian_t yaw;
        /// Pitch reported by photonvision
        units::radian_t pitch;
        /// Range from robot to target
        units::meter_t range;
    };

    /// Turn Camera LEDs on
    void TurnLEDOn();

    /// Turn Camera LEDs off
    void TurnLEDOff();

    /**
     * Returns whether or not the LEDs are on
     */
    bool IsLEDOn() const;

    /**
     * Subscribe queue to vision data
     *
     * @param queue the queue that is subscribing to recieve vision data
     */
    void SubscribeToVisionData(
        wpi::static_circular_buffer<GlobalMeasurements, 8>& queue);

    /**
     * Unsubscribe queue from the vision data
     *
     * @param queue the quue that is unsubscribing form the vision data
     */
    void UnsubscribeFromVisionData(
        wpi::static_circular_buffer<GlobalMeasurements, 8>& queue);

    /**
     * Updates vision sim data with new pose and camera transformation
     *
     * @param drivetrainPose Drivetrain pose to see if target is in range
     */
    void UpdateVisionMeasurementsSim(const frc::Pose2d& drivetrainPose);

    /**
     * Returns whether or not the camera can see a target
     */
    bool HaveTargets() const;

    /**
     * Switches the camere to driver mode
     */
    void SwitchDriverMode(bool driveSwitch);

    /**
     * Returns the value of the current pipeline index
     */
    int GetPipelineIndex();

    /**
     * Sets the pipeline of the vision system
     */
    void SetPipeline(int pipeline);

    void RobotPeriodic() override;

private:
    photonlib::PhotonCamera m_rpiCam{kVisionCamName};
    photonlib::PhotonCamera m_usbCam{kDriverCamName};

    bool m_haveTargets = false;

    std::vector<wpi::static_circular_buffer<GlobalMeasurements, 8>*>
        m_subsystemQueues;

    units::radian_t m_yaw;
    units::radian_t m_pitch;
    units::meter_t m_range;

    std::string m_allianceColor = "Blue";

    nt::NetworkTableEntry m_poseEntry = NetworkTableUtil::MakeDoubleArrayEntry(
        "/Diagnostics/Vision/Drivetrain pose");
    nt::NetworkTableEntry m_yawEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Vision/Yaw");
    nt::NetworkTableEntry m_rangeEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Vision/Range Estimate");
    nt::NetworkTableEntry m_timestampEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Vision/Timestamp");

    // Simulation variables
    photonlib::SimVisionSystem m_simVision{kVisionCamName,
                                           kCameraDiagonalFOV,
                                           kCameraPitch,
                                           frc::Transform2d{},
                                           kCameraHeight,
                                           20_m,
                                           960,
                                           720,
                                           10};
};
}  // namespace frc3512
