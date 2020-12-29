/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/scalar_multiplication.hpp"

#include "engine/core/math/types.hpp"

namespace isaac {
namespace planner_cost {

bool ScalarMultiplication::isValid(double time, const VectorXd& state) {
  if (cost_ == nullptr) return true;
  return cost_->isValid(time, state);
}

double ScalarMultiplication::evaluate(double time, const VectorXd& state) {
  if (cost_ == nullptr) return 0.0;
  return constant_ * cost_->evaluate(time, state);
}

void ScalarMultiplication::addGradient(double time, const VectorXd& state,
                                 Eigen::Ref<VectorXd> gradient) {
  if (cost_ == nullptr) return;
  VectorXd tmp_gradient = VectorXd::Zero(gradient.size());
  cost_->addGradient(time, state, tmp_gradient);
  gradient += tmp_gradient * constant_;
}

void ScalarMultiplication::addHessian(double time, const VectorXd& state,
                                      Eigen::Ref<MatrixXd> hessian) {
  if (cost_ == nullptr) return;
  MatrixXd tmp_hessian = MatrixXd::Zero(hessian.rows(), hessian.cols());
  cost_->addHessian(time, state, tmp_hessian);
  hessian += tmp_hessian * constant_;
}


}  // namespace planner_cost
}  // namespace isaac
