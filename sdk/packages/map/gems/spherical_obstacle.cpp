/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "spherical_obstacle.hpp"

#include <cmath>
#include <vector>

#include "engine/core/epsilon.hpp"

namespace isaac {
namespace map {

double SphericalObstacle::distance(const Vector2d& point) const {
  return (point - circle.center).norm() - circle.radius;
}

Vector2d SphericalObstacle::gradient(const Vector2d& point) const {
  const Vector2d direction = point - circle.center;
  const double squared_norm = direction.squaredNorm();
  // Node: this distance function has one singularity, however the gradient is pushing us away from
  // it, if this cause an error, consider changing the function to: sqrt(dx^2 + dy^2 + epsilon)
  if (IsAlmostZero(squared_norm)) {
    return Vector2d::Zero();
  }
  return direction / std::sqrt(squared_norm);
}

Matrix2d SphericalObstacle::hessian(const Vector2d& point) const {
  const Vector2d direction = point - circle.center;
  const double squared_norm = direction.squaredNorm();
  // Node: this distance function has one singularity, however the gradient is pushing us away from
  // it, if this cause an error, consider changing the function to: sqrt(dx^2 + dy^2 + epsilon)
  if (IsAlmostZero(squared_norm)) {
    return Matrix2d::Zero();
  }
  Matrix2d hessian;
  const double norm_3_2 = std::sqrt(squared_norm) * squared_norm;
  hessian(0, 0) = std::pow(direction.y(), 2) / norm_3_2;
  hessian(1, 1) = std::pow(direction.x(), 2) / norm_3_2;
  const double hxy = -direction.x() * direction.y() / norm_3_2;
  hessian(0, 1) = hxy;
  hessian(1, 0) = hxy;
  return hessian;
}

void SphericalObstacle::batchDistance(const std::vector<Vector2d>& points,
                                      std::vector<double>& distances) const {
  distances.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    distances[idx] = distance(points[idx]);
  }
}

void SphericalObstacle::batchGradient(const std::vector<Vector2d>& points,
                                      std::vector<Vector2d>& gradients) const {
  gradients.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    gradients[idx] = gradient(points[idx]);
  }
}

void SphericalObstacle::batchHessian(const std::vector<Vector2d>& points,
                                      std::vector<Matrix2d>& hessians) const {
  hessians.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    hessians[idx] = hessian(points[idx]);
  }
}

}  // namespace map
}  // namespace isaac
