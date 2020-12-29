/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/smooth_linear_target_cost.hpp"

namespace isaac {
namespace planner_cost {

double SmoothLinearTargetCost::evaluate(double /*time*/, const VectorXd& state) {
  ASSERT(target_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), target_.size());

  VectorXd delta = state - target_;
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  return gain_ * std::sqrt(squared_epsilon_ + delta.squaredNorm());
}

void SmoothLinearTargetCost::addGradient(double /*time*/, const VectorXd& state,
                                       Eigen::Ref<VectorXd> gradient) {
  ASSERT(target_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), target_.size());
  VectorXd delta = state - target_;
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  const double value = 1.0 / std::sqrt(squared_epsilon_ + delta.squaredNorm());
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  gradient += gain_ * value * delta;
}

void SmoothLinearTargetCost::addHessian(double /*time*/, const VectorXd& state,
                                      Eigen::Ref<MatrixXd> hessian) {
  ASSERT(target_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), target_.size());
  VectorXd delta = state - target_;
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
  }
  const double squared_value = squared_epsilon_ + delta.squaredNorm();
  const double squared_value_inv = 1.0 / squared_value;;
  const double value_inv = gain_ * std::sqrt(squared_value_inv);
  if (scaler_ != std::nullopt) {
    delta = delta.cwiseProduct(*scaler_);
    hessian += value_inv * scaler_->cwiseProduct(*scaler_).asDiagonal();
  } else {
    hessian += value_inv * MatrixXd::Identity(state.size(), state.size());
  }
  if (!force_positive_definite_hessian_) {
    hessian -= value_inv * squared_value_inv * delta * delta.transpose();
  }
}

}  // namespace planner_cost
}  // namespace isaac
