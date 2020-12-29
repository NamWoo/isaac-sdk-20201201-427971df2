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
#include "packages/planner_cost/gems/range_constraints_cost.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the RangeConstraintsCost evaluation.
// This component has 3 parameters that can be used to control the final cost:
// - gains that control the factor applied to each dimension.
// - min_value/max_value that describe the range where the cost is 0, while outside this range a
//   quadratic cost is applied.
// Those values can be changed live, and cost function will be updated in the next call to `update`
class RangeConstraintsCostBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Gain associated to each dimensions
  ISAAC_PARAM(VectorXd, gains);
  // Minimum value for each dimension, if the state is below this value, a quadratic cost is applied
  ISAAC_PARAM(VectorXd, min_value);
  // Maximum value for each dimension, if the state is above this value, a quadratic cost is applied
  ISAAC_PARAM(VectorXd, max_value);

 private:
  // This an implementation of the update function, it does not require the time interval as it only
  // gets the config parameter and passes them along to the cost_ function.
  void updateImpl();

  std::unique_ptr<RangeConstraintsCost> cost_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::RangeConstraintsCostBuilder);
