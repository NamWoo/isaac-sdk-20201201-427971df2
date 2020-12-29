/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// Interface for a component which builds and maintains a planner cost function. The cost function
// is created in a two stage process. `build` is used initially to create the cost function and to
// allocate memory necessary for its maintenance when calling `update`. `update` is called
// periodically to update the cost function based on latest data and does not allocate additional
// memory.
class PlannerCostBuilder : public alice::Component {
 public:
  // Creates the cost function initially. Makes sure all necessary memory required for subsequent
  // calls to `update` is allocated.
  virtual PlannerCost* build() = 0;

  // Prepares the cost function for the given time interval.
  // Does not do any dynamic memory allocations.
  virtual void update(double start_time, double end_time) {}

  // Destroys the cost function and all memory which was allocated during `build`.
  virtual void destroy() = 0;

  // Returns a pointer to the maintained cost function
  virtual PlannerCost* get() = 0;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::planner_cost::PlannerCostBuilder);
