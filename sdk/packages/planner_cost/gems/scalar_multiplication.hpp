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

// This is an implementation of PlannerCost.
// It takes another PlannerCost and simply multiplies by a constant value.
class ScalarMultiplication : public PlannerCost {
 public:
  ScalarMultiplication() = default;
  ScalarMultiplication(PlannerCost* cost, double constant) : cost_(cost), constant_(constant) {}

  bool isValid(double time, const VectorXd& state) override;
  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the PlannerCost to use.
  void setCost(PlannerCost* cost) {
    cost_ = cost;
  }

  // Sets the constant scalar to multiply.
  void setConstant(double constant) {
    constant_ = constant;
  }

 private:
  PlannerCost* cost_ = nullptr;
  double constant_ = 1.0;
};

}  // namespace planner_cost
}  // namespace isaac
