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
#include "packages/planner_cost/gems/dot_product_cost.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the DotProductCost evaluation.
// This component has 2 parameters that can be used to control the final cost:
// - `offset`: this a constant offset added to the final cost.
// - `dot_vector`: this is the vector used to compute the dot product with the state.
// Those values can be changed live, and cost function will be updated in the next call to `update`
class DotProductCostBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Constant offset to be added to the final cost.
  ISAAC_PARAM(double, offset, 0.0);
  // Vector used to compute the dot product with the state.
  ISAAC_PARAM(VectorXd, dot_vector);

 private:
  // This an implementation of the update function, it does not require the time interval as it only
  // gets the config parameter and passes them along to the cost_ function.
  void updateImpl();

  std::unique_ptr<DotProductCost> cost_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::DotProductCostBuilder);
