/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/addition.hpp"

namespace isaac {
namespace planner_cost {

bool Addition::isValid(double time, const VectorXd& state) {
  for (auto* cost : costs_) {
    if (!cost->isValid(time, state)) {
      return false;
    }
  }
  return true;
}

double Addition::evaluate(double time, const VectorXd& state) {
  double value = 0.0;
  for (auto* cost : costs_) {
    value += cost->evaluate(time, state);
  }
  return value;
}

void Addition::addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) {
  for (auto* cost : costs_) {
    cost->addGradient(time, state, gradient);
  }
}

void Addition::addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) {
  for (auto* cost : costs_) {
    cost->addHessian(time, state, hessian);
  }
}

}  // namespace planner_cost
}  // namespace isaac
