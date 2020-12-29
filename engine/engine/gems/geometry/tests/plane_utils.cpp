/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/geometry/plane_utils.hpp"

#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

namespace {

// Encode a list of points as a matrix where each column is a point.
template<typename K, int N>
Matrix<K, N, Eigen::Dynamic> PointListToMatrix(const std::vector<Vector<K, N>>& points) {
  Matrix<K, N, Eigen::Dynamic> matrix(N, points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    matrix.col(i) = points[i];
  }
  return matrix;
}

// Test that two planes are very similar.
template<typename K>
void CheckPlaneApproxEq(const Plane<K>& plane1, const Plane<K>& plane2) {
  // The normal is not uniquely defined. It can be inverted without changing the meaning.
  // So, we check that either the normal vectors match, or when one of them is negated they match.
  const float epsilon = 1e-6;
  if (std::abs(plane1.normal()[0] - plane2.normal()[0]) > epsilon ||
      std::abs(plane1.normal()[1] - plane2.normal()[1]) > epsilon ||
      std::abs(plane1.normal()[2] - plane2.normal()[2]) > epsilon) {
    // If the normal vectors do not match, try negating one of them and checking again.
    ISAAC_EXPECT_VEC_NEAR(plane1.normal(), -plane2.normal(), epsilon);
    EXPECT_NEAR(plane1.offset(), -plane2.offset(), epsilon);
  } else {
    EXPECT_NEAR(plane1.offset(), plane2.offset(), epsilon);
  }
}

}  // namespace

TEST(FitPlaneToPoints, TwoPoints) {
  std::vector<Vector3f> points {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
  };
  ASSERT_DEATH((FitPlaneToPoints<float, 3>(PointListToMatrix(points))), "underdetermined");
}

TEST(FitPlaneToPoints, ThreePoints1) {
  std::vector<Vector3f> points {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
  };
  const PlaneF computed_plane = FitPlaneToPoints<float, 3>(PointListToMatrix(points));
  const PlaneF correct_plane = PlaneF({0.0f, 0.0f, 1.0f}, 0.0f);
  CheckPlaneApproxEq(computed_plane, correct_plane);
}

TEST(FitPlaneToPoints, ThreePoints2) {
  std::vector<Vector3f> points {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f},
  };
  const PlaneF computed_plane = FitPlaneToPoints<float, 3>(PointListToMatrix(points));
  const PlaneF correct_plane = PlaneF({0.0f, 1.0f, 0.0f}, 0.0f);
  CheckPlaneApproxEq(computed_plane, correct_plane);
}

TEST(FitPlaneToPoints, ThreePoints3) {
  std::vector<Vector3f> points {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
  };
  const PlaneF computed_plane = FitPlaneToPoints<float, 3>(PointListToMatrix(points));
  const PlaneF correct_plane = PlaneF({-1.0f, 0.0f, 0.0f}, 0.0f);
  CheckPlaneApproxEq(computed_plane, correct_plane);
}

TEST(FitPlaneToPoints, ThreePoints4) {
  std::vector<Vector3f> points {
    {0.0f, 0.0f, 1.0f},
    {0.0f, 1.0f, 1.0f},
    {1.0f, 0.0f, 1.0f},
  };
  const PlaneF computed_plane = FitPlaneToPoints<float, 3>(PointListToMatrix(points));
  const PlaneF correct_plane = PlaneF({0.0f, 0.0f, 1.0f}, -1.0f);
  CheckPlaneApproxEq(computed_plane, correct_plane);
}

TEST(FitPlaneToPoints, FivePoints) {
  std::vector<Vector3f> points {
    {1.0f, 0.0f, -4.0f},
    {2.0f, -3.0f, -1.0f},
    {-1.0f, 2.0f, -2.0f},
    {5.0f, 1.0f, -18.0f},
    {0.0f, 0.0f, -1.0f},
  };
  const PlaneF computed_plane = FitPlaneToPoints<float, 3>(PointListToMatrix(points));
  const float norm = std::sqrt(14.0f);
  const PlaneF correct_plane = PlaneF(Vector3f(3.0f, 2.0f, 1.0f) / norm, 1.0f / norm);
  CheckPlaneApproxEq(computed_plane, correct_plane);
}

TEST(IterativeFitPlaneToPoints, IterativeFit) {
  std::vector<Vector3f> points;
  // Noisy points off of the plane.
  points.push_back({-0.2f, 0.6f, 1.0f});
  // Real points on the plane.
  for (int i = 0; i < 10; ++i) {
    points.push_back({0.0f, 0.0f, 0.0f});
    points.push_back({0.0f, 1.0f, 0.0f});
    points.push_back({1.0f, 0.0f, 0.0f});
  }
  points.push_back({0.5f, 0.0f, 0.0f});
  points.push_back({0.4f, 0.2f, 0.0f});
  int num_points = points.size();
  const PlaneF initial_guess({0.1f, 0.05f, 0.95f}, 0.01f);
  const double distance_threshold = 0.2;
  const double max_iterations = 10;
  std::vector<int> inliers(num_points);
  int num_inliers;
  const PlaneF computed_plane = IterativeFitPlaneToPoints<float, 3>(PointListToMatrix(points),
      initial_guess, distance_threshold, max_iterations, inliers, num_inliers);
  const PlaneF correct_plane = PlaneF({0.0f, 0.0f, 1.0f}, 0.0f);
  CheckPlaneApproxEq(computed_plane, correct_plane);
  ASSERT_EQ(num_inliers, num_points - 1);
  ASSERT_EQ(inliers.back(), 0);
  for (int i = 0; i < num_points; i++) {
    ASSERT_TRUE(std::find(inliers.begin(), inliers.end(), i) != inliers.end());
  }
}

}  // namespace geometry
}  // namespace isaac