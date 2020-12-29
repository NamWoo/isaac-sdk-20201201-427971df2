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
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/addition.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the Addition evaluation.
// This component takes a list of component names as input parameter and adds them to the list
// of parameters to be added by the cost function.
// This is done once in the build() function and won't be changed when update is called.
class AdditionBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // List of names of the components which (a PlannerCostBuilder implementation) to be added by the
  // cost function. Changes to this parameter won't be visible until build() is called again.
  ISAAC_PARAM(std::vector<std::string>, component_names);

 private:
  std::unique_ptr<Addition> cost_;
  // List of PlannerCostBuilder holding the cost function to add. This is needed to destroy them
  // when destroy is called (as component_names might have changed) as well as calling update for
  // each of them.
  std::vector<PlannerCostBuilder*> builders_;
  // Used to make sure we don't have cyclic dependencies (the builder is not adding itself).
  bool loading_ = false;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::AdditionBuilder);
