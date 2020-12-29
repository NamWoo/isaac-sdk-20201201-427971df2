/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/obstacle_distance.hpp"

#include <limits>

#include "engine/core/math/types.hpp"

namespace isaac {
namespace planner_cost {

bool ObstacleDistance::isValid(double time, const VectorXd& state) {
  return evaluate(time, state) > 0.0;
}

double ObstacleDistance::evaluate(double /*time*/, const VectorXd& state) {
  if (obstacle_ == nullptr) return std::numeric_limits<double>::max();
  const Vector2d pos = obstacle_T_state_frame_ * Vector2d(state[indices_[0]], state[indices_[1]]);
  return obstacle_->distance(pos);
}

void ObstacleDistance::addGradient(double /*time*/, const VectorXd& state,
                                   Eigen::Ref<VectorXd> gradient) {
  if (obstacle_ == nullptr) return;
  const Vector2d pos = obstacle_T_state_frame_ * Vector2d(state[indices_[0]], state[indices_[1]]);
  const Vector2d tmp_gradient =
      obstacle_T_state_frame_.rotation.matrix().transpose() * obstacle_->gradient(pos);
  gradient[indices_[0]] += tmp_gradient.x();
  gradient[indices_[1]] += tmp_gradient.y();
}

void ObstacleDistance::addHessian(double /*time*/, const VectorXd& state,
                                  Eigen::Ref<MatrixXd> hessian) {
  if (obstacle_ == nullptr) return;
  const Vector2d pos = obstacle_T_state_frame_ * Vector2d(state[indices_[0]], state[indices_[1]]);
  const Matrix2d jacobian = obstacle_T_state_frame_.rotation.matrix();
  const Matrix2d tmp_hessian = jacobian.transpose() * obstacle_->hessian(pos) * jacobian;
  hessian(indices_[0], indices_[0]) += tmp_hessian(0, 0);
  hessian(indices_[0], indices_[1]) += tmp_hessian(0, 1);
  hessian(indices_[1], indices_[0]) += tmp_hessian(1, 0);
  hessian(indices_[1], indices_[1]) += tmp_hessian(1, 1);
}

}  // namespace planner_cost
}  // namespace isaac
