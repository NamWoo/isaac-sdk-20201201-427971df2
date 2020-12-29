/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/core/math/types.hpp"

namespace isaac {
namespace map {

// Base class for an obstacle
struct Obstacle {
  virtual ~Obstacle() = default;
  // Returns the signed distance beteen a `point` and the obstacle. The absolute value corresponds
  // to the distance to the closest point on the edge of the obstacle, and a negative value means
  // the point is inside the obstacle.
  // The distance needs to be twice differentable (in order to have the gradient and hessian
  // defined).
  virtual double distance(const Vector2d& point) const = 0;
  // Returns the gradient of the distance function.
  virtual Vector2d gradient(const Vector2d& point) const = 0;
  // Returns the hessian of the distance function.
  virtual Matrix2d hessian(const Vector2d& point) const = 0;

  // Same as the distance function above, but perform a batch operation
  virtual void batchDistance(const std::vector<Vector2d>& points,
                             std::vector<double>& distances) const = 0;
  // Same as the gradient function above, but perform a batch operation
  virtual void batchGradient(const std::vector<Vector2d>& points,
                             std::vector<Vector2d>& gradient) const = 0;
  // Same as the hessian function above, but perform a batch operation
  virtual void batchHessian(const std::vector<Vector2d>& points,
                            std::vector<Matrix2d>& hessians) const = 0;
};

}  // namespace map
}  // namespace isaac
