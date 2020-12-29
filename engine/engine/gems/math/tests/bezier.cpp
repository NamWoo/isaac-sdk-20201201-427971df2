/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/gems/math/bezier.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Bezier, LinearBezier) {
  const Vector2d p0{0, 0};
  const Vector2d p1{2, 1};
  const Vector2d p2{4, 2};
  const Vector2d p3{6, 3};
  ISAAC_EXPECT_VEC_NEAR(Vector2d(0.0, 0.0), Bezier(0.0, p0, p1, p2, p3), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(Vector2d(3.0, 1.5), Bezier(0.5, p0, p1, p2, p3), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(Vector2d(6.0, 3.0), Bezier(1.0, p0, p1, p2, p3), 1e-9);
}

TEST(Bezier, LinearBezierClosestPoint) {
  const Vector2d p0{0, 0};
  const Vector2d p1{2, 1};
  const Vector2d p2{4, 2};
  const Vector2d p3{6, 3};
  EXPECT_NEAR(0.0, BezierClosestPoint(Vector2d{0.0, 0.0}, p0, p1, p2, p3), 1e-5);
  EXPECT_NEAR(0.5, BezierClosestPoint(Vector2d{3.0, 1.5}, p0, p1, p2, p3), 1e-5);
  EXPECT_NEAR(1.0, BezierClosestPoint(Vector2d{6.0, 3.0}, p0, p1, p2, p3), 1e-5);
}

TEST(Bezier, CurvedBezier) {
  const Vector2d p0{-1, +1};
  const Vector2d p1{+1, -1};
  const Vector2d p2{ 2,  2};
  const Vector2d p3{ 0,  3};
  ISAAC_EXPECT_VEC_NEAR(Vector2d(0.28125, 0.328125), Bezier(0.25, p0, p1, p2, p3), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(Vector2d(1.0    , 0.875   ), Bezier(0.50, p0, p1, p2, p3), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(Vector2d(0.96875, 1.984375), Bezier(0.75, p0, p1, p2, p3), 1e-9);
}

TEST(Bezier, CurvedBezierClosestPoint) {
  const Vector2d p0{-1, +1};
  const Vector2d p1{+1, -1};
  const Vector2d p2{ 2,  2};
  const Vector2d p3{ 0,  3};
  EXPECT_NEAR(0.25, BezierClosestPoint(Vector2d(0.28125, 0.328125), p0, p1, p2, p3), 1e-4);
  EXPECT_NEAR(0.50, BezierClosestPoint(Vector2d(1.0    , 0.875   ), p0, p1, p2, p3), 1e-4);
  EXPECT_NEAR(0.75, BezierClosestPoint(Vector2d(0.96875, 1.98438 ), p0, p1, p2, p3), 1e-4);
}

}  // namespace isaac
