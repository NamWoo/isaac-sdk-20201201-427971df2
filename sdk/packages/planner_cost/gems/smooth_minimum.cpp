/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/smooth_minimum.hpp"

#include <algorithm>
#include <cmath>

namespace isaac {
namespace planner_cost {

bool SmoothMinimum::isValid(double time, const VectorXd& state) {
  for (auto* cost : costs_) {
    if (!cost->isValid(time, state)) {
      return false;
    }
  }
  return true;
}

double SmoothMinimum::evaluate(double time, const VectorXd& state) {
  if (costs_.empty()) return 0.0;
  if (costs_.size() == 1) return costs_.front()->evaluate(time, state);
  values_.clear();
  for (auto* cost : costs_) {
    values_.push_back(cost->evaluate(time, state));
  }
  // Find the default value to extract to make it more numericably stable
  const double default_value = minimum_smoothing_ > 0.0
      ? *std::min_element(values_.begin(), values_.end())
      : *std::max_element(values_.begin(), values_.end());
  double sum_exp = 0.0;
  for (double value : values_) {
    sum_exp += std::exp(-minimum_smoothing_ * (value - default_value));
  }
  return default_value - std::log(sum_exp) / minimum_smoothing_;
}

void SmoothMinimum::addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) {
  if (costs_.empty()) return;
  if (costs_.size() == 1) {
    costs_.front()->addGradient(time, state, gradient);
    return;
  }
  // Extract all the value
  values_.clear();
  for (auto* cost : costs_) {
    values_.push_back(cost->evaluate(time, state));
  }
  // Find the default value to extract to make it more numericably stable
  const double default_value = minimum_smoothing_ > 0.0
      ? *std::min_element(values_.begin(), values_.end())
      : *std::max_element(values_.begin(), values_.end());
  double sum_exp = 0.0;
  VectorXd sum_gradient = VectorXd::Zero(gradient.size());
  VectorXd tmp_gradient(gradient.size());
  // The gradient of the smooth function is the sum of the weighted gradient.
  for (size_t i = 0; i < values_.size(); i++) {
    const double weight = std::exp(-minimum_smoothing_ * (values_[i] - default_value));
    sum_exp += weight;
    tmp_gradient.setZero();
    costs_[i]->addGradient(time, state, tmp_gradient);
    sum_gradient += weight * tmp_gradient;
  }
  gradient += sum_gradient / sum_exp;
}

void SmoothMinimum::addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) {
  if (costs_.empty()) return;
  if (costs_.size() == 1) {
    costs_.front()->addHessian(time, state, hessian);
    return;
  }
  // Compute the list of value
  values_.clear();
  for (auto* cost : costs_) {
    values_.push_back(cost->evaluate(time, state));
  }
  // Find the default value to extract to make it more numericably stable
  const double default_value = minimum_smoothing_ > 0.0
      ? *std::min_element(values_.begin(), values_.end())
      : *std::max_element(values_.begin(), values_.end());
  // Compute the various needed sum
  double sum_exp = 0.0;
  VectorXd sum_gradient = VectorXd::Zero(state.size());
  VectorXd tmp_gradient(state.size());
  MatrixXd sum_hessian = MatrixXd::Zero(state.size(), state.size());
  MatrixXd sum_gradient_product = MatrixXd::Zero(state.size(), state.size());
  MatrixXd tmp_hessian(state.size(), state.size());
  for (size_t i = 0; i < values_.size(); i++) {
    const double weight = std::exp(-minimum_smoothing_ * (values_[i] - default_value));
    sum_exp += weight;
    // Compute gradient for each cost
    tmp_gradient.setZero();
    costs_[i]->addGradient(time, state, tmp_gradient);
    sum_gradient += weight * tmp_gradient;
    sum_gradient_product += weight * tmp_gradient * tmp_gradient.transpose();
    // Compute hessian for each cost
    tmp_hessian.setZero();
    costs_[i]->addHessian(time, state, tmp_hessian);
    sum_hessian += weight * tmp_hessian;
  }
  // The final hessian is:
  // hessian = sum_weighted_hessian
  //         + alpha (sum_weighted_gradient * sum_weighted_gradient^T -
  //                  sum_weighted_gradient_product)
  const double sum_exp_inv = 1.0 / sum_exp;
  sum_hessian += minimum_smoothing_ *
                (sum_exp_inv * sum_gradient * sum_gradient.transpose() - sum_gradient_product);
  hessian += sum_hessian * sum_exp_inv;
}

}  // namespace planner_cost
}  // namespace isaac
