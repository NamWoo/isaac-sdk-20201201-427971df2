/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/polygon_speed_limit.hpp"

#include <algorithm>

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/smooth_distance.hpp"

namespace isaac {
namespace planner_cost {

double PolygonSpeedLimit::evaluate(double /*time*/, const VectorXd& state) {
  // Quick check to see if the speed is already below the speed limit, in which case the cost will
  // be 0.0 and we can avoid checking the distance to the polygon.
  const double speed_delta = std::abs(state[indices_[2]]) - speed_limit_;
  if (speed_delta <= 0.0) return 0.0;
  // We look if we are inside the area
  const Vector2d position(state[indices_[0]], state[indices_[1]]);
  for (const auto& polygon : polygons_) {
    if (!polygon.isInside(position)) continue;
    const double distance = geometry::SmoothDistanceToPolyline(polygon.points, position);
    // We compute the final cost
    const double distance_squared = distance * distance;
    return 0.5 * gain_ * speed_delta * speed_delta * distance_squared /
                     (distance_squared + squared_sigma_);
  }
  return 0.0;
}

void PolygonSpeedLimit::addGradient(double /*time*/, const VectorXd& state,
                                    Eigen::Ref<VectorXd> gradient) {
  // Quick check to see if the speed is already below the speed limit, in which case the cost will
  // be 0.0 and we can avoid checking the distance to the polygon.
  const double speed = state[indices_[2]];
  const double speed_delta = std::abs(speed) - speed_limit_;
  if (speed_delta <= 0.0) return;
  const double signed_speed_delta = speed < 0.0 ? - speed_delta : speed_delta;
  const Vector2d position(state[indices_[0]], state[indices_[1]]);
  for (const auto& polygon : polygons_) {
    // We look if we are inside the area
    if (!polygon.isInside(position)) continue;
    Vector2d grad;
    const double distance = geometry::SmoothDistanceToPolyline(polygon.points, position, &grad);
    // We update the gradient.
    const double distance_squared = distance * distance;
    const double tot_sum = distance_squared + squared_sigma_;
    const double tot_sum_inv = 1.0 / tot_sum;
    const double update = gain_ * speed_delta * speed_delta * distance * squared_sigma_ *
                          tot_sum_inv * tot_sum_inv;
    gradient[indices_[0]] += update * grad[0];
    gradient[indices_[1]] += update * grad[1];
    gradient[indices_[2]] += gain_ * distance_squared * signed_speed_delta * tot_sum_inv;
    break;
  }
}

void PolygonSpeedLimit::addHessian(double /*time*/, const VectorXd& state,
                                   Eigen::Ref<MatrixXd> hessian) {
  // Quick check to see if the speed is already below the speed limit, in which case the cost will
  // be 0.0 and we can avoid checking the distance to the polygon.
  const double speed = state[indices_[2]];
  const double speed_delta = std::abs(speed) - speed_limit_;
  if (speed_delta <= 0.0) return;
  const double signed_speed_delta = speed < 0.0 ? - speed_delta : speed_delta;
  const Vector2d position(state[indices_[0]], state[indices_[1]]);
  for (const auto& polygon : polygons_) {
    // We look if we are inside the area
    if (!polygon.isInside(position)) continue;
    Vector2d grad;
    const double distance = geometry::SmoothDistanceToPolyline(polygon.points, position, &grad);
    // We update the hessian.
    const double distance_squared = distance * distance;
    const double tot_sum = distance_squared + squared_sigma_;
    const double tot_sum_inv = 1.0 / tot_sum;
    const double update = gain_ * signed_speed_delta * squared_sigma_ * tot_sum_inv * tot_sum_inv;

    // This is the part of the hessian which is safe ()
    double update_xy =
        update * signed_speed_delta * (1.0 - 4.0 * distance * distance * tot_sum_inv);

    // The real hessian is not be positive definite, depending on the gain, this might dominate the
    // final hessian and create issue in the LQR solver, one solution is to ignore the part of the
    // hessian that creates the issue.
    if (force_positive_definite_hessian_) {
      update_xy = std::max(update_xy, 0.0);
    } else {
      const double update_vx = 2.0 * update * distance;
      hessian(indices_[0], indices_[2]) += update_vx * grad[0];
      hessian(indices_[1], indices_[2]) += update_vx * grad[1];
      hessian(indices_[2], indices_[0]) += update_vx * grad[0];
      hessian(indices_[2], indices_[1]) += update_vx * grad[1];
    }
    hessian(indices_[0], indices_[0]) += update_xy * grad[0] * grad[0];
    hessian(indices_[0], indices_[1]) += update_xy * grad[0] * grad[1];
    hessian(indices_[1], indices_[0]) += update_xy * grad[1] * grad[0];
    hessian(indices_[1], indices_[1]) += update_xy * grad[1] * grad[1];

    hessian(indices_[2], indices_[2]) += gain_ * distance_squared * tot_sum_inv;
    break;
  }
}

}  // namespace planner_cost
}  // namespace isaac
