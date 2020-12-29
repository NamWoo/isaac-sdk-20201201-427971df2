/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/polygon_distance_quadratic_cost.hpp"

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/smooth_distance.hpp"

namespace isaac {
namespace planner_cost {

double PolygonDistanceQuadraticCost::evaluate(double /*time*/, const VectorXd& state) {
  const double distance = geometry::SmoothDistanceToPolyline(
      polyline_, Vector2d(state[indices_[0]], state[indices_[1]])) + offset_;
  if (distance <= 0.0) return 0.0;
  return 0.5 * gain_ * distance * distance;
}

void PolygonDistanceQuadraticCost::addGradient(double /*time*/, const VectorXd& state,
                                               Eigen::Ref<VectorXd> gradient) {
  Vector2d grad;
  const double distance = geometry::SmoothDistanceToPolyline(
      polyline_, Vector2d(state[indices_[0]], state[indices_[1]]), &grad) + offset_;
  if (distance < 0.0) return;
  gradient[indices_[0]] += gain_ * distance * grad[0];
  gradient[indices_[1]] += gain_ * distance * grad[1];
}

void PolygonDistanceQuadraticCost::addHessian(double /*time*/, const VectorXd& state,
                                              Eigen::Ref<MatrixXd> hessian) {
  Vector2d grad;
  const double distance = geometry::SmoothDistanceToPolyline(
      polyline_, Vector2d(state[indices_[0]], state[indices_[1]]), &grad);
  if (distance < 0.0) return;
  const double update_xy = gain_ * grad[0] * grad[1];
  hessian(indices_[0], indices_[0]) += gain_ * grad[0] * grad[0];
  hessian(indices_[0], indices_[1]) += update_xy;
  hessian(indices_[1], indices_[0]) += update_xy;
  hessian(indices_[1], indices_[1]) += gain_ * grad[1] * grad[1];
}

}  // namespace planner_cost
}  // namespace isaac
