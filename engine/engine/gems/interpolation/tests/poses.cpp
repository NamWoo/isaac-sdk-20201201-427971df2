/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/interpolation/poses.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(MathPoseUtils, InterpolatePose2Translation) {
  Pose2d a = Pose2d::Identity();
  Pose2d b = Pose2d::Translation(Vector2d{1.0, -2.7});
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.0, a, b), a, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.1, a, b),
                         Pose2d::Translation(Vector2d{0.1, -0.27}), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(1.0, a, b), b, 1e-12);
}

TEST(MathPoseUtils, InterpolatePose3Translation) {
  Pose3d a = Pose3d::Identity();
  Pose3d b = Pose3d::Translation(Vector3d{1.0, -2.7, 0.3});
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.0, a, b), a, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.1, a, b),
                         Pose3d::Translation(Vector3d{0.1, -0.27, 0.03}), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(1.0, a, b), b, 1e-12);
}

TEST(MathPoseUtils, InterpolatePose2Rotation) {
  Pose2d a = Pose2d::Identity();
  Pose2d b = Pose2d::Rotation(1.7);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.0, a, b), a, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.1, a, b), Pose2d::Rotation(0.17), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(1.0, a, b), b, 1e-12);
}

TEST(MathPoseUtils, InterpolatePose3Rotation) {
  Pose3d a = Pose3d::Identity();
  Pose3d b = Pose3d::Rotation(Vector3d{1.0, -2.7, 0.3}, 1.7);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.0, a, b), a, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(0.1, a, b),
                         Pose3d::Rotation(Vector3d{1.0, -2.7, 0.3}, 0.17), 1e-12);
  ISAAC_EXPECT_POSE_NEAR(Interpolate(1.0, a, b), b, 1e-12);
}

}  // namespace isaac
