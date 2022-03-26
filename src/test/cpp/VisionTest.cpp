// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>

#include "Constants.hpp"
#include "NetworkTableUtil.hpp"
#include "SimulatorTest.hpp"
#include "TargetModel.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Vision.hpp"

class VisionTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc3512::Vision vision;
};

TEST_F(VisionTest, GetData) {
    photonlib::PhotonCamera rpiCam{frc3512::Vision::kVisionCamName};
    const frc::Pose2d drivetrainPose{12.89_m, 2.41_m,
                                     units::radian_t{wpi::numbers::pi}};

    // Simulation variables
    photonlib::SimVisionSystem simVision{frc3512::Vision::kVisionCamName,
                                         frc3512::Vision::kCameraDiagonalFOV,
                                         frc3512::Vision::kCameraPitch,
                                         frc::Transform2d{},
                                         frc3512::Vision::kCameraHeight,
                                         20_m,
                                         960,
                                         720,
                                         10};

    frc::Pose2d kTargetPose{
        frc::Translation2d{TargetModel::kCenter.X(), TargetModel::kCenter.Y()},
        0_rad};
    photonlib::SimVisionTarget newTgt{kTargetPose, 2.606548_m, 1.2192_m,
                                      2.606548_m};
    simVision.AddSimVisionTarget(newTgt);

    simVision.MoveCamera(frc::Transform2d{frc::Translation2d{},
                                          units::radian_t{wpi::numbers::pi}},
                         frc3512::Vision::kCameraHeight,
                         frc3512::Vision::kCameraPitch);
    simVision.ProcessFrame(drivetrainPose);

    // Flush and delay to ensure value propagates to tables
    nt::NetworkTableInstance::GetDefault().Flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    photonlib::PhotonPipelineResult result = rpiCam.GetLatestResult();

    ASSERT_TRUE(result.HasTargets());
}
