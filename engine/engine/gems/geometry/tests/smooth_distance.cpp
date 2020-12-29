/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/geometry/smooth_distance.hpp"

#include "engine/gems/geometry/polyline.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

TEST(SmoothDistance, simple_test) {
  Vector2d gradient;
  Polyline2d polyline;
  polyline.push_back(Vector2d(-20.0, -30));
  polyline.push_back(Vector2d(0.0, -10));
  polyline.push_back(Vector2d(0.0, 10));
  polyline.push_back(Vector2d(20.0, 30));
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.0, 0.0), &gradient), 0.0, 1e-9);
  EXPECT_NEAR(gradient[0], -1.0, 5e-5);
  EXPECT_NEAR(gradient[1], 0.0, 5e-5);
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.5, 0.0), &gradient), -0.5, 5e-3);
  EXPECT_NEAR(gradient[0], -1.0, 1e-2);
  EXPECT_NEAR(gradient[1], 0.0, 1e-2);
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(-0.5, 0.0), &gradient), 0.5, 5e-3);
  EXPECT_NEAR(gradient[0], -1.0, 1e-2);
  EXPECT_NEAR(gradient[1], 0.0, 1e-2);
}

TEST(SmoothDistance, single_points) {
  Vector2d gradient;
  Polyline2d polyline;
  polyline.push_back(Vector2d(3.0, -4.0));
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.0, 0.0), &gradient), 5.0, 1e-9);
  EXPECT_NEAR(gradient[0], -0.6, 5e-5);
  EXPECT_NEAR(gradient[1], 0.8, 5e-5);
}

TEST(SmoothDistance, empty_polyline) {
  Vector2d gradient;
  Polyline2d polyline;
  EXPECT_EQ(SmoothDistanceToPolyline(polyline, Vector2d(13.0, -42.0), &gradient), 0.0);
  EXPECT_EQ(gradient[0], 0.0);
  EXPECT_EQ(gradient[1], 0.0);
}

TEST(SmoothDistance, polygon_clockwise) {
  Vector2d gradient;
  Polyline2d polyline;
  polyline.push_back(Vector2d(-10.0, -10.0));
  polyline.push_back(Vector2d(-10.0, 10.0));
  polyline.push_back(Vector2d(10.0, 10.0));
  polyline.push_back(Vector2d(10.0, -10.0));
  polyline.push_back(Vector2d(-10.0, -10.0));
  EXPECT_LT(SmoothDistanceToPolyline(polyline, Vector2d(9.0, 0.0), &gradient), -0.5);
  EXPECT_GT(SmoothDistanceToPolyline(polyline, Vector2d(11.0, 0.0), &gradient), 0.5);
  EXPECT_GT(SmoothDistanceToPolyline(polyline, Vector2d(-10.0, -11.0), &gradient), -0.5);
  EXPECT_GT(SmoothDistanceToPolyline(polyline, Vector2d(-11.0, -10.0), &gradient), -0.5);
}

TEST(SmoothDistance, polygon_anticlockwise) {
  Vector2d gradient;
  Polyline2d polyline;
  polyline.push_back(Vector2d(-10.0, -10.0));
  polyline.push_back(Vector2d(10.0, -10.0));
  polyline.push_back(Vector2d(10.0, 10.0));
  polyline.push_back(Vector2d(-10.0, 10.0));
  polyline.push_back(Vector2d(-10.0, -10.0));
  EXPECT_GT(SmoothDistanceToPolyline(polyline, Vector2d(9.0, 0.0), &gradient), 0.5);
  EXPECT_LT(SmoothDistanceToPolyline(polyline, Vector2d(11.0, 0.0), &gradient), -0.5);
  EXPECT_LT(SmoothDistanceToPolyline(polyline, Vector2d(-11.0, -10.0), &gradient), -0.5);
  EXPECT_LT(SmoothDistanceToPolyline(polyline, Vector2d(-10.0, -11.0), &gradient), -0.5);
}

TEST(SmoothDistance, on_the_line) {
  Vector2d gradient;
  Polyline2d polyline;
  polyline.push_back(Vector2d(0.0, -10));
  polyline.push_back(Vector2d(0.0, 10));
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.0, 0.0), &gradient), 0.0, 1e-3);
  EXPECT_NEAR(gradient[0], -1.0, 5e-3);
  EXPECT_NEAR(gradient[1], 0.0, 5e-3);
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.0, -10.0), &gradient), 0.0, 1e-3);
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.0, 10.0), &gradient), 0.0, 1e-3);
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.0, 20.0), &gradient), 0.0, 1e-3);
  EXPECT_NEAR(SmoothDistanceToPolyline(polyline, Vector2d(0.0, -20.0), &gradient), 0.0, 1e-3);
}

}  // namespace geometry
}  // namespace isaac
