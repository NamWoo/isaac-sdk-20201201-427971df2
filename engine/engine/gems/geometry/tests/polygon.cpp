/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/gems/geometry/polygon.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

TEST(IsInside, convex) {
  std::vector<Vector2d> points;
  points.push_back(Vector2d(1.0, 1.0));
  points.push_back(Vector2d(2.0, 0.0));
  points.push_back(Vector2d(2.0, -1.0));
  points.push_back(Vector2d(0.0, -2.0));
  points.push_back(Vector2d(-1.0, 0.0));
  points.push_back(Vector2d(0.0, 1.0));
  Polygon2D poly{points};

  EXPECT_TRUE(poly.isInside(Vector2d(0.0, 0.0)));
  EXPECT_TRUE(poly.isInside(Vector2d(0.5, 0.5)));
  EXPECT_TRUE(poly.isInside(Vector2d(1.4999, 0.5)));
  EXPECT_TRUE(poly.isInside(Vector2d(-0.7999, 0.2)));
  EXPECT_TRUE(poly.isInside(Vector2d(1.0, 0.9999)));

  // Obvious out
  EXPECT_FALSE(poly.isInside(Vector2d(5.0, 0.0)));
  EXPECT_FALSE(poly.isInside(Vector2d(-5.0, 0.0)));
  EXPECT_FALSE(poly.isInside(Vector2d(0.0, -5.0)));
  EXPECT_FALSE(poly.isInside(Vector2d(0.0, 5.0)));
  // Close out
  EXPECT_FALSE(poly.isInside(Vector2d(1.50001, 0.5)));
  EXPECT_FALSE(poly.isInside(Vector2d(-0.8001, 0.2)));
  EXPECT_FALSE(poly.isInside(Vector2d(1.0, 1.00001)));
}

TEST(slowIsInside, convex) {
  std::vector<Vector2d> points;
  points.push_back(Vector2d(1.0, 1.0));
  points.push_back(Vector2d(2.0, 0.0));
  points.push_back(Vector2d(2.0, -1.0));
  points.push_back(Vector2d(0.0, -2.0));
  points.push_back(Vector2d(-1.0, 0.0));
  points.push_back(Vector2d(0.0, 1.0));
  Polygon2D poly{points};

  EXPECT_TRUE(poly.slowIsInside(Vector2d(0.0, 0.0)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(0.5, 0.5)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(1.4999, 0.5)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(-0.7999, 0.2)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(1.0, 0.9999)));

  // Obvious out
  EXPECT_FALSE(poly.slowIsInside(Vector2d(5.0, 0.0)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(-5.0, 0.0)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(0.0, -5.0)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(0.0, 5.0)));
  // Close out
  EXPECT_FALSE(poly.slowIsInside(Vector2d(1.50001, 0.5)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(-0.8001, 0.2)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(1.0, 1.00001)));
}

TEST(isInside, concave) {
  std::vector<Vector2d> points;
  points.push_back(Vector2d(1.0, 1.0));
  points.push_back(Vector2d(2.0, 0.0));
  points.push_back(Vector2d(1.0, 0.0));
  points.push_back(Vector2d(2.0, -1.0));
  points.push_back(Vector2d(1.0, -1.0));
  points.push_back(Vector2d(0.0, 0.0));
  points.push_back(Vector2d(0.0, -1.0));
  points.push_back(Vector2d(-1.0, 0.0));
  points.push_back(Vector2d(-1.0, 1.0));
  Polygon2D poly{points};

  // Check around the singularity point
  EXPECT_TRUE(poly.isInside(Vector2d(0.0, -0.01)));
  EXPECT_FALSE(poly.isInside(Vector2d(5.e-15, -0.01)));
  EXPECT_TRUE(poly.isInside(Vector2d(-5.e-15, -0.01)));
  EXPECT_TRUE(poly.isInside(Vector2d(-0.01, 0.0)));

  EXPECT_TRUE(poly.isInside(Vector2d(0.01, 0.01)));
  EXPECT_TRUE(poly.isInside(Vector2d(-0.01, 0.01)));
  EXPECT_TRUE(poly.isInside(Vector2d(-0.01, -0.01)));
  EXPECT_TRUE(poly.isInside(Vector2d(0.02, 0.01)));
  EXPECT_FALSE(poly.isInside(Vector2d(0.01, -0.02)));
}

TEST(slowIsInside, concave) {
  std::vector<Vector2d> points;
  points.push_back(Vector2d(1.0, 1.0));
  points.push_back(Vector2d(2.0, 0.0));
  points.push_back(Vector2d(1.0, 0.0));
  points.push_back(Vector2d(2.0, -1.0));
  points.push_back(Vector2d(1.0, -1.0));
  points.push_back(Vector2d(0.0, 0.0));
  points.push_back(Vector2d(0.0, -1.0));
  points.push_back(Vector2d(-1.0, 0.0));
  points.push_back(Vector2d(-1.0, 1.0));
  Polygon2D poly{points};

  // Check around the singularity point
  EXPECT_TRUE(poly.slowIsInside(Vector2d(0.0, -0.01)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(5.e-15, -0.01)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(-5.e-15, -0.01)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(-0.01, 0.0)));

  EXPECT_TRUE(poly.slowIsInside(Vector2d(0.01, 0.01)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(-0.01, 0.01)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(-0.01, -0.01)));
  EXPECT_TRUE(poly.slowIsInside(Vector2d(0.02, 0.01)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(0.01, -0.02)));
}

TEST(distance, convex) {
  std::vector<Vector2d> points;
  points.push_back(Vector2d(1.0, 1.0));
  points.push_back(Vector2d(2.0, 0.0));
  points.push_back(Vector2d(2.0, -1.0));
  points.push_back(Vector2d(0.0, -2.0));
  points.push_back(Vector2d(-1.0, 0.0));
  points.push_back(Vector2d(0.0, 1.0));
  Polygon2D poly{points};

  EXPECT_NEAR(-std::sqrt(0.5), poly.signedDistance(Vector2d(0.0, 0.0)), 1e-7);
  EXPECT_NEAR(-0.5, poly.signedDistance(Vector2d(0.5, 0.5)), 1e-7);
  EXPECT_NEAR(0.0, poly.signedDistance(Vector2d(-0.8, 0.2)), 1e-7);
  EXPECT_NEAR(0.0, poly.signedDistance(Vector2d(1.0, 1.0)), 1e-7);

  // Obvious out
  EXPECT_NEAR(3.0, poly.signedDistance(Vector2d(5.0, 0.0)), 1e-7);
  EXPECT_NEAR(4.0, poly.signedDistance(Vector2d(-5.0, 0.0)), 1e-7);
  EXPECT_NEAR(3.0, poly.signedDistance(Vector2d(0.0, -5.0)), 1e-7);
  EXPECT_NEAR(4.0, poly.signedDistance(Vector2d(0.0, 5.0)), 1e-7);
}

TEST(IsInside, diamond) {
  std::vector<Vector2d> points;
  points.push_back(Vector2d(0.0, 100.0));
  points.push_back(Vector2d(100.0, 0.0));
  points.push_back(Vector2d(0.0, -100.0));
  points.push_back(Vector2d(-100.0, 0.0));
  Polygon2D poly{points};

  EXPECT_TRUE(poly.isInside(Vector2d(0.0, 0.0)));
  EXPECT_TRUE(poly.isInside(Vector2d(9.033173505792323e-16, -7.222975850023629e-16)));
  EXPECT_TRUE(poly.isInside(Vector2d(-9.033173505792323e-16, -7.222975850023629e-16)));
  EXPECT_TRUE(poly.isInside(Vector2d(-9.033173505792323e-16, 7.222975850023629e-16)));
  EXPECT_TRUE(poly.isInside(Vector2d(9.033173505792323e-16, 7.222975850023629e-16)));
}

TEST(IsInside, square) {
  std::vector<Vector2d> points;
  points.push_back(Vector2d(2.0, 2.0));
  points.push_back(Vector2d(2.0, -2.0));
  points.push_back(Vector2d(-2.0, -2.0));
  points.push_back(Vector2d(-2.0, 2.0));
  Polygon2D poly{points};
  // Those should fallback to slowIsInside.
  EXPECT_TRUE(poly.isInside(Vector2d(1.0, 0.0)));
  EXPECT_TRUE(poly.isInside(Vector2d(1.0, -7.222975850023629e-16)));
  EXPECT_TRUE(poly.isInside(Vector2d(1.0, +7.222975850023629e-16)));
  EXPECT_TRUE(poly.isInside(Vector2d(1.0-7.222975850023629e-15, 0.0)));
  EXPECT_TRUE(poly.isInside(Vector2d(1.0+7.222975850023629e-15, 0.0)));
}

TEST(inside, empty_polygon) {
  Polygon2D poly{};
  EXPECT_FALSE(poly.isInside(Vector2d(1.0, 0.0)));
  EXPECT_FALSE(poly.slowIsInside(Vector2d(1.0, 0.0)));
}

}  // namespace geometry
}  // namespace isaac
