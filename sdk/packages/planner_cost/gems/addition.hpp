/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// Computes the sum of a list of PlannerCost.
// You can add a list of PlannerCost using addCost function and the output of this function will
// simply be the sum. The gradient and hessian can be easily derivated as being the sum of the
// gradients and hessians for each function inside costs_.
class Addition : public PlannerCost {
 public:
  Addition() = default;

  // This PlannerCost considers a state as valid if each cost function that depends on it consider
  // it as valid.
  bool isValid(double time, const VectorXd& state) override;
  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Adds a PlannerCost to the list.
  void addCost(PlannerCost* cost) {
    if (cost == nullptr) return;
    costs_.push_back(cost);
  }

 private:
  // Store the list of cost function to use when evaluating this function. They will all be added.
  std::vector<PlannerCost*> costs_;
};

}  // namespace planner_cost
}  // namespace isaac
