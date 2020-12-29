/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/geometry/pinhole.hpp"

#include "engine/core/constants.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

TEST(Pinhole, FromHorizontalFieldOfView) {
  const auto pinhole = Pinhole<double>::FromHorizontalFieldOfView(100, 200, DegToRad(90.0));
  EXPECT_EQ(pinhole.dimensions[0], 100);
  EXPECT_EQ(pinhole.dimensions[1], 200);
  EXPECT_NEAR(pinhole.focal[0], 100.0, 1e-12);
  EXPECT_NEAR(pinhole.focal[1], 100.0, 1e-12);
  EXPECT_NEAR(pinhole.center[0], 50.0, 1e-12);
  EXPECT_NEAR(pinhole.center[1], 100.0, 1e-12);
}

TEST(Pinhole, FromVerticalFieldOfView) {
  const auto pinhole = Pinhole<double>::FromVerticalFieldOfView(120, 231, DegToRad(90.0));
  EXPECT_EQ(pinhole.dimensions[0], 120);
  EXPECT_EQ(pinhole.dimensions[1], 231);
  EXPECT_NEAR(pinhole.focal[0], 60.0, 1e-12);
  EXPECT_NEAR(pinhole.focal[1], 60.0, 1e-12);
  EXPECT_NEAR(pinhole.center[0], 60.0, 1e-12);
  EXPECT_NEAR(pinhole.center[1], 115.5, 1e-12);
}

TEST(Pinhole, Project) {
  const auto pinhole = Pinhole<double>::FromVerticalFieldOfView(120, 231, DegToRad(90.0));
  const Vector3d expected{0.7, -1.1, 2.33};
  const Vector2d pixel = pinhole.project(expected);
  const Vector3d actual = pinhole.unproject(pixel, expected.z());
  ISAAC_EXPECT_VEC_NEAR(expected, actual, 1e-9);
}

TEST(Pinhole, CropAndScale) {
  const Pinhole<double> pinhole{{720, 1280}, {700.0,   700.0}, {360.0, 640.0}};
  const Pinhole<double> expected{{257, 513}, {299.833333333333333333333333333333333, 299.25}, {128.5, 256.5}};
  const auto actual = pinhole.cropAndScale(Vector2i{60, 40}, Vector2i{600, 1200}, Vector2i{257, 513});
  EXPECT_EQ(expected.dimensions[0], actual.dimensions[0]);
  EXPECT_EQ(expected.dimensions[1], actual.dimensions[1]);
  ISAAC_EXPECT_VEC_NEAR(expected.focal, actual.focal, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(expected.center, actual.center, 1e-9);
}

TEST(Pinhole, Scale) {
  const Pinhole<double> pinhole{{720, 1280}, {700.0,   700.0}, {360.0, 640.0}};
  const Pinhole<double> expected{{360, 320}, {350.0, 175.0}, {180.0, 160.0}};
  const auto actual =
      pinhole.scale(Vector2d{static_cast<double>(360.0) / static_cast<double>(720.0),
                             static_cast<double>(320.0) / static_cast<double>(1280.0)});
  EXPECT_EQ(expected.dimensions[0], actual.dimensions[0]);
  EXPECT_EQ(expected.dimensions[1], actual.dimensions[1]);
  ISAAC_EXPECT_VEC_NEAR(expected.focal, actual.focal, 1e-9);
  ISAAC_EXPECT_VEC_NEAR(expected.center, actual.center, 1e-9);
}

}  // namespace geometry
}  // namespace isaac
