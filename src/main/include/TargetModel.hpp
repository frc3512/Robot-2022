// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>

namespace TargetModel {
// An offset is added to the real target location so the robot aims for a point
// a few inches in above of the target.
extern const frc::Translation2d kOffset;
static constexpr frc::Translation3d kA{629.25_in, 0_in, 105.362_in};
static constexpr frc::Translation3d kB{629.25_in, 53.75_in, 101.362_in};
static constexpr frc::Translation3d kCenter = (kA + kB) / 2.0;
extern const frc::Pose2d kTargetPoseInGlobal;
}  // namespace TargetModel
