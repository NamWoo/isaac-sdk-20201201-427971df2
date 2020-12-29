/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/types.hpp"
#include "packages/map/gems/obstacle.hpp"

namespace isaac {
namespace map {

// A spherical obstacle at a certain position.
// The distance is 0 on the boundary of the object and negative inside. The minimum is at the center
// of the circle, and the distance is -radius.
struct SphericalObstacle : public Obstacle {
  // The position and radius of the obstacle
  geometry::CircleD circle;

  double distance(const Vector2d& point) const  override;

  Vector2d gradient(const Vector2d& point) const  override;

  Matrix2d hessian(const Vector2d& point) const  override;

  void batchDistance(const std::vector<Vector2d>& points,
                     std::vector<double>& distances) const  override;

  void batchGradient(const std::vector<Vector2d>& points,
                     std::vector<Vector2d>& gradients) const  override;

  void batchHessian(const std::vector<Vector2d>& points,
                     std::vector<Matrix2d>& hessians) const  override;
};

}  // namespace map
}  // namespace isaac
