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
// This compute a function that behaves like a quadratic function close to 0, but is capped to a
// maximum value. The actual formula is:
//    maximum * distance^2 / (sigma^2 + distance^2)
// or an equivalent formulation:
//    maximum * (1.0 - sigma^2 / (sigma^2 + distance^2))
// Maximum is the maximum value this function can take, sigma controls the range where the function
// behaves as a quadratic function, while distance is the L2 norm of the difference from the state
// to the target.
class BoundedQuadraticCost : public PlannerCost {
 public:
  BoundedQuadraticCost() = default;

  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the maximum value this function can returns
  void setMaximum(double maximum) {
    maximum_ = maximum;
  }

  // Sets the sigma value (see description above to see how it is used in the formula)
  void setSigma(double sigma) {
    squared_sigma_ = sigma * sigma;
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
  bool force_positive_definite_hessian_ = true;
  double squared_sigma_ = 0.1;
  double maximum_ = 1.0;
  VectorXd target_;
  std::optional<VectorXd> scaler_ = std::nullopt;
};

}  // namespace planner_cost
}  // namespace isaac
