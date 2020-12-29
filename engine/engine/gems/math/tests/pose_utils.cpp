/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/gems/math/pose_utils.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(PoseUtils, LookAtOrigin) {
  // make sure it doesn't crash
  ASSERT_NO_THROW(LookAt(Vector3d{0.0, 0.0, 0.0}));
}

TEST(PoseUtils, LookAt) {
  const Vector3d forward{1.0, 0.0, 0.0};
  ISAAC_EXPECT_SO_NEAR_ID(LookAt(forward), 1e-9);
  Vector3d target{-1.0, 0.0, 0.0};
  ISAAC_EXPECT_VEC_NEAR(LookAt(target)*forward, target, 1e-9);
  target = {0.0, 1.0, 0.0};
  ISAAC_EXPECT_VEC_NEAR(LookAt(target)*forward, target, 1e-9);
  target = {0.0, -1.0, 0.0};
  ISAAC_EXPECT_VEC_NEAR(LookAt(target)*forward, target, 1e-9);
  target = {0.0, 0.0, 1.0};
  ISAAC_EXPECT_VEC_NEAR(LookAt(target)*forward, target, 1e-9);
  target = {0.0, 0.0, -1.0};
  ISAAC_EXPECT_VEC_NEAR(LookAt(target)*forward, target, 1e-9);
  target = {0.5, 0.5, -1.0};
  ISAAC_EXPECT_VEC_NEAR(LookAt(target)*forward, target.normalized(), 1e-9);
}

}  // namespace isaac