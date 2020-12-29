/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "Eigen/Eigen"
#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/plane.hpp"

namespace isaac {
namespace geometry {

// Returns the plane which best fits the point cloud.
// In the input matrix, points, each column is an N-dimensional point in a point cloud.
// We require at least N points to fit a plane.
template<typename K, int N>
Hyperplane<K, N> FitPlaneToPoints(const Eigen::Ref<const Matrix<K, N, Eigen::Dynamic>>& points) {
  ASSERT(points.cols() >= N, "insufficient data, problem is underdetermined");

  // Calculate mean of each coordinate and subtract it to center the data.
  Vector<K, N> mean;
  for (size_t i = 0; i < N; ++i) {
    mean[i] = points.row(i).mean();
  }
  Matrix<K, N, Eigen::Dynamic> X(N, points.cols());
  for (int i = 0; i < X.cols(); ++i) {
    X.col(i) = points.col(i) - mean;
  }

  // Use SVD to find the best fitting plane.
  const auto svd = X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Vector<K, N> plane_normal = svd.matrixU().rightCols(1);

  // The mean is always part of the plane. We project this point onto the normal to find the
  // offset of the plane.
  const K offset = -plane_normal.dot(mean);
  return Hyperplane<K, N>(plane_normal, offset);
}

// Iteratively fit a plane to a point cloud using an initial guess.
// This algorithm is more robust to outliers than just fitting a plane to all of the points because
// it iteratively discards outliers and tries to fit a plane to only the inliers.
// In each iteration, we only keep points which are closer to the plane than a specified threshold.
// Then after discarding all of the outliers, the plane is refit.
// This is repeated until there are fewer than N points, the algorithm converges on a set of points,
// or max_iterations is reached.
template<typename K, int N>
Hyperplane<K, N> IterativeFitPlaneToPoints(
    const Eigen::Ref<const Matrix<K, N, Eigen::Dynamic>>& initial_points,
    const Hyperplane<K, N>& initial_guess, double distance_threshold, int max_iterations,
    std::vector<int>& inliers, int& num_inliers) {
  ASSERT(max_iterations > 0, "must have at least one iteration");
  int num_points = initial_points.cols();
  ASSERT(num_points >= N, "insufficient data, cannot fit a plane");
  ASSERT(distance_threshold >= 0, "distance threshold must be nonnegative");
  ISAAC_ASSERT_EQ(num_points, static_cast<int>(inliers.size()));

  // The plane and points which are updated at each iteration.
  Hyperplane<K, N> plane = initial_guess;
  Matrix<K, N, Eigen::Dynamic> points = initial_points;
  for (int i = 0; i < num_points; i++) {
    inliers[i] = i;
  }

  // Keep track of the previous number of points considered.
  // Then, if we try to consider the same set of points again, we know we have converged.
  int prev_num_cols = 0;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Remove all the points which are further from the plane than the threshold for this iteration.
    // We still keep them around in the array so that they can be considered on future iterations.
    // Each time through this loop we either increase i by 1 or decrease num_cols by 1 and move a
    // point which we have already considered to the end of the points matrix. By doing this, we
    // ensure that every point is tested exactly once on each iteration.
    int num_cols = points.cols();
    int first_swap_index = num_cols;
    for (int i = 0; i < num_cols; ++i) {
      const K distance = plane.absDistance(points.col(i));
      if (distance > distance_threshold) {
        // Copy in another point, and increase the count of points to remove at the end.
        points.col(i).swap(points.col(num_cols - 1));
        std::swap(inliers[i], inliers[num_cols - 1]);
        first_swap_index = std::min(first_swap_index, i);
        i--;
        num_cols--;
      }
    }

    // After the first iteration, if we have the same number of points as we did before,
    // and we do not swap any of these points out, then we are considering the same set of
    // points. Hence, we have converged and can early out here.
    if (iter > 0 && prev_num_cols == num_cols && first_swap_index >= num_cols) break;

    if (num_cols < N) break;  // Not enough data to fit a plane.
    plane = FitPlaneToPoints<K, N>(points.block(0, 0, N, num_cols));
    prev_num_cols = num_cols;
  }
  num_inliers = prev_num_cols;

  return plane;
}

}  // namespace geometry
}  // namespace isaac
