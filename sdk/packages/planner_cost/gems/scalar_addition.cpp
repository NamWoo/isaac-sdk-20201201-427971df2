/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/scalar_addition.hpp"

#include "engine/core/math/types.hpp"

namespace isaac {
namespace planner_cost {

bool ScalarAddition::isValid(double time, const VectorXd& state) {
  if (cost_ == nullptr) return true;
  return cost_->isValid(time, state);
}

double ScalarAddition::evaluate(double time, const VectorXd& state) {
  if (cost_ == nullptr) return constant_;
  return constant_ + cost_->evaluate(time, state);
}

void ScalarAddition::addGradient(double time, const VectorXd& state,
                                 Eigen::Ref<VectorXd> gradient) {
  if (cost_ == nullptr) return;
  cost_->addGradient(time, state, gradient);
}

void ScalarAddition::addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) {
  if (cost_ == nullptr) return;
  cost_->addHessian(time, state, hessian);
}


}  // namespace planner_cost
}  // namespace isaac
