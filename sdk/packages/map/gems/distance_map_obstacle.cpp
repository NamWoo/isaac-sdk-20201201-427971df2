/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "distance_map_obstacle.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace {
// How many pixel to add on each side to fix the distance outside the image
constexpr size_t kMargin = 3;
constexpr double kMarginDouble = static_cast<double>(kMargin);
}  // namespace

namespace isaac {
namespace map {

DistanceMapObstacle::DistanceMapObstacle(Image1d&& distance_image, Parameters parameters)
    : parameters_(parameters), distance_image_(std::move(distance_image)) {
  ASSERT(parameters_.cell_size > 0.0, "cell size needs to be positive: %lf", parameters_.cell_size);
  cell_size_inv_ = 1.0 / parameters_.cell_size;
  squared_cell_size_inv_ = cell_size_inv_ * cell_size_inv_;
  if (parameters_.outside_distance) {
    distance_bicubic_ = std::make_unique<BicubicApproximatedFunction<double>>(
        [this](size_t row, size_t col) {
          if (row < kMargin || row >= kMargin + distance_image_.rows() ||
              col < kMargin || col >= kMargin + distance_image_.cols()) {
            return *parameters_.outside_distance;
          }
          return distance_image_(row - kMargin, col - kMargin);
        }, distance_image_.rows() + 2 * kMargin, distance_image_.cols() + 2 * kMargin);
  } else {
    distance_bicubic_ = std::make_unique<BicubicApproximatedFunction<double>>(
        [this](size_t row, size_t col) {
          return distance_image_(row, col);
        }, distance_image_.rows(), distance_image_.cols());
  }
}

double DistanceMapObstacle::distance(const Vector2d& point) const {
  const double margin = parameters_.outside_distance ? kMarginDouble : 0.0;
  return distance_bicubic_->get(point.x() * cell_size_inv_ + margin,
                                point.y() * cell_size_inv_ + margin);
}

Vector2d DistanceMapObstacle::gradient(const Vector2d& point) const {
  const double margin = parameters_.outside_distance ? kMarginDouble : 0.0;
  const Vector2d grad =
      distance_bicubic_->gradient(point.x() * cell_size_inv_ + margin,
                                  point.y() * cell_size_inv_ + margin);
  return grad * cell_size_inv_;
}

Matrix2d DistanceMapObstacle::hessian(const Vector2d& point) const {
  const double margin = parameters_.outside_distance ? kMarginDouble : 0.0;
  Matrix2d hess = distance_bicubic_->hessian(point.x() * cell_size_inv_ + margin,
                                             point.y() * cell_size_inv_ + margin);
  return hess * squared_cell_size_inv_;
}

void DistanceMapObstacle::batchDistance(const std::vector<Vector2d>& points,
                                        std::vector<double>& distances) const {
  // TODO(ben): provide GPU accelerated implementation.
  distances.resize(points.size());
  for (size_t idx = 0; idx < points.size(); idx++) {
    distances[idx] = distance(points[idx]);
  }
}

void DistanceMapObstacle::batchGradient(const std::vector<Vector2d>& points,
                                        std::vector<Vector2d>& gradients) const {
  gradients.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    gradients[idx] = gradient(points[idx]);
  }
}

void DistanceMapObstacle::batchHessian(const std::vector<Vector2d>& points,
                                       std::vector<Matrix2d>& hessians) const {
  hessians.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    hessians[idx] = hessian(points[idx]);
  }
}

}  // namespace map
}  // namespace isaac
