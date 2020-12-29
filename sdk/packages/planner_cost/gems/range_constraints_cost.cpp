/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/range_constraints_cost.hpp"

namespace isaac {
namespace planner_cost {

double RangeConstraintsCost::evaluate(double /*time*/, const VectorXd& state) {
  ASSERT(gains_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), gains_.size());
  double value = 0.0;
  for (int i = 0; i < state.size(); i++) {
    if (gains_[i] == 0.0) continue;
    if (state[i] < min_value_[i]) {
      const double delta = min_value_[i] - state[i];
      value += gains_[i] * delta * delta;
    }
    if (state[i] > max_value_[i]) {
      const double delta = max_value_[i] - state[i];
      value += gains_[i] * delta * delta;
    }
  }
  return 0.5 * value;
}

void RangeConstraintsCost::addGradient(double /*time*/, const VectorXd& state,
                                       Eigen::Ref<VectorXd> gradient) {
  ASSERT(gains_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), gains_.size());
  for (int i = 0; i < state.size(); i++) {
    if (gains_[i] == 0.0) continue;
    if (state[i] < min_value_[i]) {
      gradient[i] += (state[i] - min_value_[i]) * gains_[i];
    }
    if (state[i] > max_value_[i]) {
      gradient[i] += (state[i] - max_value_[i]) * gains_[i];
    }
  }
}

void RangeConstraintsCost::addHessian(double /*time*/, const VectorXd& state,
                                      Eigen::Ref<MatrixXd> hessian) {
  ASSERT(gains_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), gains_.size());
  for (int i = 0; i < state.size(); i++) {
    if (gains_[i] == 0.0) continue;
    if (state[i] < min_value_[i]) {
      hessian(i, i) += gains_[i];
    }
    if (state[i] >= max_value_[i]) {
      hessian(i, i) += gains_[i];
    }
  }
}

}  // namespace planner_cost
}  // namespace isaac
