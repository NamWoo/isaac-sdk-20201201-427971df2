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
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// Compute a quadratic cost if the state is outside the provided range:
// This function has 3 parameters:
//   - min_value: the threshold below which we start adding a quadratic penalty
//   - max_value: the threshold above which we start adding a quadratic penalty
//   - gains: the gain associated to each dimension of the state
// eval(state) = 0.5 * sum(g_i * (min(s_i - min_i, 0)^2 + max(s_i - max_i, 0)^2))
class RangeConstraintsCost : public PlannerCost {
 public:
  RangeConstraintsCost() = default;

  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the gain associated to each dimension
  void setGains(const VectorXd& gains) {
    ASSERT(gains_.size() == 0 || gains_.size() == gains.size(),
           "The expected dimension of state cannot be changed from %d to %d",
           gains_.size(), gains.size());
    gains_ = gains;
  }

  // Sets the threshold below which we start adding a quadratic penalty
  void setMinValue(const VectorXd& min_value) {
    ASSERT(min_value_.size() == 0 || min_value_.size() == min_value.size(),
           "The expected dimension of state cannot be changed from %d to %d",
           min_value_.size(), min_value.size());
    min_value_ = min_value;
  }

  // Sets the threshold above which we start adding a quadratic penalty
  void setMaxValue(const VectorXd& max_value) {
    ASSERT(max_value_.size() == 0 || max_value_.size() == max_value.size(),
           "The expected dimension of state cannot be changed from %d to %d",
           max_value_.size(), max_value.size());
    max_value_ = max_value;
  }

  // Accessor to the current gains
  VectorXd& gains() { return gains_; }
  const VectorXd& gains() const { return gains_; }

  // Accessor to the current min_value
  VectorXd& min_value() { return min_value_; }
  const VectorXd& min_value() const { return min_value_; }

  // Accessor to the current max_value
  VectorXd& max_value() { return max_value_; }
  const VectorXd& max_value() const { return max_value_; }

 private:
  VectorXd gains_;
  VectorXd min_value_;
  VectorXd max_value_;
};

}  // namespace planner_cost
}  // namespace isaac
