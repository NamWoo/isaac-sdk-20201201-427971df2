/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// This a PlannerCost implementation:
// It takes a target state and compute the L2 distance to this state. Unfortunately the L2 norm does
// not behave nicely toward 0. So instead of using the norm directly, we will compute an
// approximation of the norm using an approximation of the absolute value function:
// abs(x) ~= sqrt(epsilon^2 + x^2)
// When epsilon goes toward zero, the approximation gets better.
// This function is convex (which is nice for optimization problem), differentiable as many time as
// we want.
class SmoothLinearTargetCost : public PlannerCost {
 public:
  SmoothLinearTargetCost() = default;

  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the gain
  void setGain(double gain) {
    gain_ = gain;
  }

  // Sets the epsilon value (see description above to see how it is used in the formula)
  void setEpsilon(double epsilon) {
    squared_epsilon_ = epsilon * epsilon;
  }

  // Sets the threshold above which we start adding a quadratic penalty
  void setTarget(const VectorXd& target) {
    ASSERT(target_.size() == 0 || target_.size() == target.size(),
           "The expected dimension of state cannot be changed from %d to %d",
           target_.size(), target.size());
    target_ = target;
  }

  // Sets the scaler: instead of computing the normal L2 norm, we will rescale each dimension by
  // a factor. This allow disabling some of the dimension or penalizing more others.
  void setScaler(const std::optional<VectorXd>& scaler) {
    scaler_ = scaler;
  }

  // Whether or not we force the hessian to be positive definite. For optimization problem it might
  // be important for the hessian to be positive definite. If this parameter is set to true, we will
  // approximate the hessian by a form which is positive definite.
  void setForcePositiveDefiniteHessian(bool force_positive_definite_hessian) {
    force_positive_definite_hessian_ = force_positive_definite_hessian;
  }

  // Accessor to the current target
  VectorXd& target() { return target_; }
  const VectorXd& target() const { return target_; }

 private:
  bool force_positive_definite_hessian_ = false;
  double squared_epsilon_ = 0.1;
  double gain_ = 1.0;
  VectorXd target_;
  std::optional<VectorXd> scaler_ = std::nullopt;
};

}  // namespace planner_cost
}  // namespace isaac
