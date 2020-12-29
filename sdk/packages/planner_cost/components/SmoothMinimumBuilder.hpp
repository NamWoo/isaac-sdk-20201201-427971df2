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
#include "packages/planner_cost/gems/smooth_minimum.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the SmoothMinimum evaluation.
// This component takes a list of component names as input parameter and will compute a smooth
// approximation of the minimum.
// The list of components to load is read only inside the build() function. No update will be
// visible until the PlannerCost is build again.
class SmoothMinimumBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // List of names of the components (a PlannerCostBuilder implementation) to be used by the cost
  // function. Changes to this parameter won't be visible until build() is called again.
  ISAAC_PARAM(std::vector<std::string>, component_names);

  // Smoothing factor used inside this function. This controls how close to the minimum the function
  // will be. The expected difference between the minimum and the evaluation of cost_ will be lower
  // than ln(builders_.size()) / minimum_smoothing.
  // If minimum_smoothing is negative, then this function be changed into a smooth maximum.
  // This value needs to be different from 0.0.
  ISAAC_PARAM(double, minimum_smoothing, 20.0);

 private:
  std::unique_ptr<SmoothMinimum> cost_;
  // List of PlannerCostBuilder holding the cost function to be used. This is needed to destroy them
  // when destroy is called (as component_names might have changed) as well as calling update for
  // each of them.
  std::vector<PlannerCostBuilder*> builders_;
  // Used to make sure we don't have cyclic dependencies (the builder is not adding itself).
  bool loading_ = false;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::SmoothMinimumBuilder);
