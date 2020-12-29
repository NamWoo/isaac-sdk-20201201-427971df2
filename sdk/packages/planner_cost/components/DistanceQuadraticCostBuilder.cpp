/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/DistanceQuadraticCostBuilder.hpp"

#include <string>

namespace isaac {
namespace planner_cost {

PlannerCost* DistanceQuadraticCostBuilder::build() {
  builder_ = node()->app()->findComponentByName<PlannerCostBuilder>(get_component_name());
  ASSERT(builder_ != nullptr,
         "Failed to load the component: %s", get_component_name().c_str());
  distance_quadratic_cost_.reset(new DistanceQuadraticCost());
  distance_quadratic_cost_->setDistanceFunction(builder_->build());
  return static_cast<PlannerCost*>(distance_quadratic_cost_.get());
}

void DistanceQuadraticCostBuilder::update(double start_time, double end_time) {
  builder_->update(start_time, end_time);
  distance_quadratic_cost_->setIndices(get_indices_mapping());
  distance_quadratic_cost_->resetCostFunctions();
  for (const Vector3d& cost : get_costs()) {
    distance_quadratic_cost_->addCostFunction(cost[0], cost[1], cost[2]);
  }
}

void DistanceQuadraticCostBuilder::destroy() {
  distance_quadratic_cost_.reset();
  builder_->destroy();
}

PlannerCost* DistanceQuadraticCostBuilder::get() {
  return static_cast<PlannerCost*>(distance_quadratic_cost_.get());
}

}  // namespace planner_cost
}  // namespace isaac
