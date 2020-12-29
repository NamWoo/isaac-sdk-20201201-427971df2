/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/map/gems/distance_map_obstacle.hpp"
#include "packages/map/gems/polygon_obstacle.hpp"
#include "packages/map/gems/spherical_obstacle.hpp"

#include <random>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/image/distance_map.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace map {

// For a given obstacle and given position, we check the gradient/hessian approximate enough the
// distance in all the direction.
void CheckDerivative(const Obstacle* obstacle, const Vector2d& point) {
  const double dist = obstacle->distance(point);
  const Vector2d grad = obstacle->gradient(point);
  const Matrix2d hessian = obstacle->hessian(point);
  constexpr double kEpsilon = 1e-3;
  constexpr double kPrecision = 2e-5;
  for (double angle = 0.0; angle < 2.0 * Pi<double>; angle += Pi<double> * 0.1) {
    const Vector2d dx = SO2d::FromAngle(angle) * Vector2d(kEpsilon, 0.0);
    const double real_dist = obstacle->distance(point + dx);
    const double approximate_dist = dist + dx.dot(grad) + 0.5 * dx.transpose() * hessian * dx;
    EXPECT_NEAR(approximate_dist, real_dist, kPrecision);
   }
 }

TEST(SphericalObstacle, Derivative) {
  SphericalObstacle obstacle;
  std::default_random_engine rng(313);
  std::uniform_real_distribution<double> pose_range(-10.0, 10.0);
  std::uniform_real_distribution<double> radius_range(0.0, 10.0);
  for (int circle = 0; circle < 100; circle++) {
    obstacle.circle.center.x() = pose_range(rng);
    obstacle.circle.center.y() = pose_range(rng);
    obstacle.circle.radius = radius_range(rng);
    for (int pt = 0; pt < 10000; pt++){
      CheckDerivative(&obstacle, Vector2d(pose_range(rng), pose_range(rng)));
    }
  }
}

TEST(PolygonObstacle, Derivative) {
  geometry::Polygon2D poly;
  poly.points.push_back(Vector2d(1.0, 1.0));
  poly.points.push_back(Vector2d(-1.0, 1.0));
  poly.points.push_back(Vector2d(-1.0, -2.0));
  poly.points.push_back(Vector2d(0.0, -1.0));
  poly.points.push_back(Vector2d(2.0, -2.0));
  poly.points.push_back(Vector2d(4.0, 0.0));
  poly.points.push_back(Vector2d(0.5, 0.5));

  PolygonObstacle obstacle(poly);
  std::default_random_engine rng(313);
  std::uniform_real_distribution<double> pose_range(-10.0, 10.0);
  for (int pt = 0; pt < 100000; pt++){
    CheckDerivative(&obstacle, Vector2d(pose_range(rng), pose_range(rng)));
  }
}

TEST(PolygonObstacle, Outside) {
  geometry::Polygon2D poly;
  poly.points.push_back(Vector2d(1.0, 1.0));
  poly.points.push_back(Vector2d(-1.0, 1.0));
  poly.points.push_back(Vector2d(-1.0, -2.0));
  poly.points.push_back(Vector2d(0.0, -1.0));
  poly.points.push_back(Vector2d(2.0, -2.0));
  poly.points.push_back(Vector2d(4.0, 0.0));
  poly.points.push_back(Vector2d(0.5, 0.5));

  constexpr double kMaxDistance = 1.0;
  PolygonObstacle obstacle(poly, {0.1, kMaxDistance});
  for (double x = -20.0; x < 20.0; x += 0.5) {
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(x, 20.0)));
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(x, -20.0)));
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(20.0, x)));
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(-20.0, x)));
  }
}

TEST(PolygonObstacle, MultiPolygon) {
  geometry::Polygon2D poly;
  poly.points.push_back(Vector2d(1.0, 1.0));
  poly.points.push_back(Vector2d(-1.0, 1.0));
  poly.points.push_back(Vector2d(-1.0, -2.0));
  poly.points.push_back(Vector2d(0.0, -1.0));
  poly.points.push_back(Vector2d(2.0, -2.0));
  poly.points.push_back(Vector2d(4.0, 0.0));
  poly.points.push_back(Vector2d(0.5, 0.5));
  geometry::Polygon2D poly2;
  for (auto& pt : poly.points) {
    poly2.points.push_back(pt + Vector2d(-2.5, 8.3));
  }

  PolygonObstacle obstacle({poly, poly2});
  std::default_random_engine rng(313);
  std::uniform_real_distribution<double> pose_range(-10.0, 10.0);
  for (int pt = 0; pt < 100000; pt++){
    CheckDerivative(&obstacle, Vector2d(pose_range(rng), pose_range(rng)));
  }
}

TEST(DistanceMapObstacle, Derivative) {
  std::default_random_engine rng(313);
  std::uniform_real_distribution<double> cell_size_range(0.01, 10.0);
  constexpr int kSize = 512;
  std::uniform_int_distribution<int> coordinates(0, 512 - 1);
  for (int test = 0; test < 100; test++) {
    const double cell_size = cell_size_range(rng);
    std::uniform_real_distribution<double> pose_range(0.0, kSize * cell_size);
    // Create a random map with enough obstacle in the middle, and some random point
    Image1ub map(kSize, kSize);
    for (int idx = 0; idx < map.num_pixels(); idx++) {
      map[idx] = 0;
    }
    for (int row = 200; row < 300; row++) {
      for (int col = 200; col < 300; col++) {
        map(row, col) = 255;
      }
    }

    for (int rand = 0; rand < 100; rand++) {
      int row = coordinates(rng);
      int col = coordinates(rng);
      map(row, col) = 255 - map(row, col);
    }

    const Image1d distance_map_in = QuickDistanceMap<1000>(map, 0, cell_size, 1e9);
    const Image1d distance_map_out = QuickDistanceMap<1000>(map, 255, cell_size, 1e9);
    Image1d distance_map(kSize, kSize);
    for (int idx = 0; idx < distance_map.num_pixels(); idx++) {
      if (map[idx] == 255) {
        distance_map[idx] = cell_size - distance_map_in[idx];
      } else {
        distance_map[idx] = distance_map_out[idx];
      }
    }
    DistanceMapObstacle obstacle(std::move(distance_map), cell_size);
    for (int pt = 0; pt < 10000; pt++){
      CheckDerivative(&obstacle, Vector2d(pose_range(rng), pose_range(rng)));
    }
  }
}

TEST(DistanceMapObstacle, Outside) {
  constexpr double kCellSize = 0.1;
  constexpr int kSize = 512;
  Image1ub map(kSize, kSize);
  for (int idx = 0; idx < map.num_pixels(); idx++) {
    map[idx] = 0;
  }
  for (int row = 200; row < 300; row++) {
    for (int col = 200; col < 300; col++) {
      map(row, col) = 255;
    }
  }

  const Image1d distance_map_in = QuickDistanceMap<1000>(map, 0, kCellSize, 1e9);
  const Image1d distance_map_out = QuickDistanceMap<1000>(map, 255, kCellSize, 1e9);
  Image1d distance_map(kSize, kSize);

  for (int idx = 0; idx < distance_map.num_pixels(); idx++) {
    if (map[idx] == 255) {
      distance_map[idx] = kCellSize - distance_map_in[idx];
    } else {
      distance_map[idx] = distance_map_out[idx];
    }
  }

  const double kMaxDistance = 10.0;
  DistanceMapObstacle obstacle(std::move(distance_map), {0.1, kMaxDistance});
  for (double x = -200.0; x < 200.0; x += 0.5) {
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(x, 200.0)));
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(x, -200.0)));
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(200.0, x)));
    ASSERT_EQ(kMaxDistance, obstacle.distance(Vector2d(-200.0, x)));
  }
}

}  // namespace map
}  // namespace isaac
