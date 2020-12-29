/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/geometry/convex_hull.hpp"

#include <memory>
#include <random>

#include "engine/core/constants.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

// Check that 2 points are enough to return a convex hull
TEST(line, two_points) {
  std::vector<Vector2d> clouds = {
    {1.0, 1.0},
    {-1.0, -1.0},
  };
  const auto hull = ConvexHull(std::move(clouds));
  ASSERT_EQ(hull.points.size(), 2);
}

// Check that if 3 points are aligned, only the extremities are returned
TEST(line, three_points) {
  std::vector<Vector2d> clouds = {
    {1.0, 1.0},
    {0.0, 0.0},
    {-1.0, -1.0},
  };
  const auto hull = ConvexHull(std::move(clouds));
  ASSERT_EQ(hull.points.size(), 2);
  EXPECT_GE(hull.points[0].squaredNorm(), 0.5);
  EXPECT_GE(hull.points[1].squaredNorm(), 0.5);
}

// Check that point on the convex hull edges are removed.
TEST(square, square) {
  std::vector<Vector2d> clouds = {
    {1.0, 1.0},
    {0.0, 1.0},
    {-1.0, 1.0},
    {-1.0, 0.0},
    {-1.0, -1.0},
    {0.0, -1.0},
    {1.0, -1.0},
    {1.0, 0.0},
  };
  const auto hull = ConvexHull(std::move(clouds));
  ASSERT_EQ(hull.points.size(), 4);
  for (int i = 0; i < 4; i++) {
    EXPECT_GE(hull.points[i].squaredNorm(), 1.5);
  }
}

// Random test, generate many cloud of points of various size and check each point is
// inside the returned polygon (signed distance <= 0).
TEST(square, random) {
  std::mt19937 rng(1337);
  std::uniform_real_distribution<double> dis_radius(0.0, 10.0);
  std::uniform_real_distribution<double> dis_angle(-Pi<double>, Pi<double>);
  for (int test = 0; test < 1000; test++) {
    std::vector<Vector2d> clouds;
    for (int i = 0; i <= test; i++) {
      const double angle = dis_angle(rng);
      const double radius = dis_radius(rng);
      clouds.push_back(radius * Vector2d(std::cos(angle), std::sin(angle)));
    }
    const auto hull = ConvexHull(clouds);
    for (const auto& pt : clouds) {
      EXPECT_LE(hull.signedDistance(pt), 0.0);
    }
  }
}

}  // namespace geometry
}  // namespace isaac
