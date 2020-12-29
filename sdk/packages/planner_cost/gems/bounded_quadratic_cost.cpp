/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/bounded_quadratic_cost.hpp"

namespace isaac {
namespace planner_cost {

double BoundedQuadraticCost::evaluate(double /*time*/, const VectorXd& state) {
  ASSERT(target_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), target_.size());

  VectorXd delta = state - target_;
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  const double squared_distance = delta.squaredNorm();
  return maximum_ * squared_distance / (squared_sigma_ + squared_distance);
}

void BoundedQuadraticCost::addGradient(double /*time*/, const VectorXd& state,
                                       Eigen::Ref<VectorXd> gradient) {
  ASSERT(target_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), target_.size());
  VectorXd delta = state - target_;
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  const double squared_value = squared_sigma_ + delta.squaredNorm();
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  gradient += 2.0 * maximum_ * squared_sigma_ / (squared_value * squared_value) * delta;
}

void BoundedQuadraticCost::addHessian(double /*time*/, const VectorXd& state,
                                      Eigen::Ref<MatrixXd> hessian) {
  ASSERT(target_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), target_.size());
  VectorXd delta = state - target_;
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  const double squared_value = squared_sigma_ + delta.squaredNorm();
  const double squared_value_inv = 1.0 / squared_value;

  const double factor = 2.0 * maximum_ * squared_sigma_ * squared_value_inv * squared_value_inv;
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
    hessian += factor * scaler_->cwiseProduct(*scaler_).asDiagonal();
  } else {
    hessian += factor * MatrixXd::Identity(state.size(), state.size());
  }
  if (!force_positive_definite_hessian_) {
    hessian -= 4.0 * factor * squared_value_inv * delta * delta.transpose();
  }
}

}  // namespace planner_cost
}  // namespace isaac
