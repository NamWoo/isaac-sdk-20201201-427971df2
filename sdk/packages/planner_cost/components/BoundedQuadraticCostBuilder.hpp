/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>

#include "engine/alice/alice_codelet.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/bounded_quadratic_cost.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the BoundedQuadraticCost evaluation.
// The cost is: maximum * (1.0 - sigma^2 / (sigma^2 + (scaler * (state-target)).squaredNorm()).
// This component has 4 parameters that can be used to control the final cost:
// - `maximum` that control the factor applied to the final cost
// - `target` used to compute the cost (see formula above)
// - `sigma` controls the spread where the function behave as a quadratic function
// - `scaler` used to control how much each dimension contribute to the cost (it's optional).
// Those values can be changed live, and cost function will be updated in the next call to `update`
class BoundedQuadraticCostBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Scaler apply on each dimension when computed (start - target). This parameter is optional, by
  // default each dimension will contribute equally.
  ISAAC_PARAM(VectorXd, scaler);
  // Target state, the error is computed as a difference from the state to the target.
  ISAAC_PARAM(VectorXd, target);
  // controls the spread where the function behave as a quadratic function.
  ISAAC_PARAM(double, sigma, 1.1);
  // Set the maximum value this function can take
  ISAAC_PARAM(double, maximum, 1.0);
  // Whether or not we force the hessian to be positive definite. For optimization problem it might
  // be important for the hessian to be positive definite. If this parameter is set to true, we will
  // approximate the hessian by a form which is positive definite.
  ISAAC_PARAM(bool, force_positive_definite_hessian, false);

 private:
  // This an implementation of the update function, it does not require the time interval as it only
  // gets the config parameter and passes them along to the cost_ function.
  void updateImpl();

  std::unique_ptr<BoundedQuadraticCost> cost_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::BoundedQuadraticCostBuilder);
