/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/math/pose_average.hpp"

#include "engine/core/constants.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace math {

TEST(PoseAverage2, Basic) {
  Pose2AverageD averager;
  averager.add(Pose2d{SO2d::FromAngle(-0.3), Vector2d{-1.1, +0.7}});
  averager.add(Pose2d{SO2d::FromAngle(+0.3), Vector2d{+1.1, -0.7}});
  auto maybe_pose = averager.computeAverage();
  ASSERT_TRUE(maybe_pose);
  ISAAC_EXPECT_POSE_NEAR_ID(*maybe_pose, 1e-9);
}

TEST(PoseAverage2, Empty) {
  Pose2AverageD averager;
  ASSERT_FALSE(averager.computeAverage());
}

TEST(PoseAverage2, Antipodal) {
  Pose2AverageD averager;
  averager.add(Pose2d::Rotation(0));
  averager.add(Pose2d::Rotation(Pi<double>));
  auto maybe_pose = averager.computeAverage();
  ASSERT_TRUE(maybe_pose);
  ISAAC_EXPECT_POSE_NEAR_ID(*maybe_pose, 1e-9);
}

}  // namespace math
}  // namespace isaac
