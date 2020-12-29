/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/math/so2.hpp"

#include <vector>

#include "gtest/gtest.h"

namespace isaac {

TEST(SO2, composition) {
  SO2d rot1 = SO2d::FromAngle(1.1);
  SO2d rot2 = SO2d::FromAngle(1.7);
  SO2d rot3 = SO2d::FromAngle(2.5);
  SO2d rot4 = SO2d::FromAngle(0.535);
  EXPECT_NEAR(rot1.angle(), 1.1, 1e-7);
  EXPECT_NEAR(rot2.angle(), 1.7, 1e-7);
  EXPECT_NEAR(rot3.angle(), 2.5, 1e-7);
  EXPECT_NEAR(rot4.angle(), 0.535, 1e-7);
  EXPECT_NEAR((rot1 * rot2 * rot3 * rot4).angle(),
              SO2d::FromAngle(rot1.angle() + rot2.angle() + rot3.angle() + rot4.angle()).angle(),
              1e-7);
}

TEST(SO2, angle) {
  SO2d rot1 = SO2d::FromAngle(1.1);
  SO2d rot2 = SO2d::FromAngle(1.7);
  SO2d rot3 = SO2d::FromAngle(2.5);
  SO2d rot4 = SO2d::FromAngle(0.535);
  EXPECT_NEAR(rot1.angle(), 1.1, 1e-7);
  EXPECT_NEAR(rot2.angle(), 1.7, 1e-7);
  EXPECT_NEAR(rot3.angle(), 2.5, 1e-7);
  EXPECT_NEAR(rot4.angle(), 0.535, 1e-7);
}

TEST(SO2, inverse) {
  SO2d rot1 = SO2d::FromAngle(1.1);
  SO2d rot2 = SO2d::FromAngle(1.7);
  SO2d rot3 = SO2d::FromAngle(2.5);
  SO2d rot4 = SO2d::FromAngle(0.535);
  EXPECT_NEAR(rot1.inverse().angle(), -1.1, 1e-7);
  EXPECT_NEAR(rot2.inverse().angle(), -1.7, 1e-7);
  EXPECT_NEAR(rot3.inverse().angle(), -2.5, 1e-7);
  EXPECT_NEAR(rot4.inverse().angle(), -0.535, 1e-7);
}

TEST(SO2, vector2) {
  SO2d rot = SO2d::FromDirection(1.0, 1.0);
  Vector2d vec1(2.0, 0.0);
  Vector2d vec2 = rot * vec1;
  Vector2d vec3 = rot * vec2;
  Vector2d vec4 = rot * vec3;
  Vector2d vec5 = rot * vec4;
  Vector2d vec6 = rot * vec5;
  Vector2d vec7 = rot * vec6;
  Vector2d vec8 = rot * vec7;
  Vector2d vec9 = rot * vec8;
  const double sqrt2 = std::sqrt(2.0);
  EXPECT_NEAR(vec2.x(), sqrt2, 1e-7);
  EXPECT_NEAR(vec2.y(), sqrt2, 1e-7);

  EXPECT_NEAR(vec3.x(), 0.0, 1e-7);
  EXPECT_NEAR(vec3.y(), 2.0, 1e-7);

  EXPECT_NEAR(vec4.x(), -sqrt2, 1e-7);
  EXPECT_NEAR(vec4.y(), sqrt2, 1e-7);

  EXPECT_NEAR(vec5.x(), -2.0, 1e-7);
  EXPECT_NEAR(vec5.y(), 0.0, 1e-7);

  EXPECT_NEAR(vec6.x(), -sqrt2, 1e-7);
  EXPECT_NEAR(vec6.y(), -sqrt2, 1e-7);

  EXPECT_NEAR(vec7.x(), 0.0, 1e-7);
  EXPECT_NEAR(vec7.y(), -2.0, 1e-7);

  EXPECT_NEAR(vec8.x(), sqrt2, 1e-7);
  EXPECT_NEAR(vec8.y(), -sqrt2, 1e-7);

  EXPECT_NEAR(vec9.x(), 2.0, 1e-7);
  EXPECT_NEAR(vec9.y(), 0.0, 1e-7);
}

}  // namespace isaac
