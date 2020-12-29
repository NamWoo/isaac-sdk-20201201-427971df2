/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/SmoothMinimumBuilder.hpp"

#include <string>

namespace isaac {
namespace planner_cost {

PlannerCost* SmoothMinimumBuilder::build() {
  ASSERT(!loading_, "The SmoothMinimumBuilder has itself in its list of dependencies. "
                    "This creates an infinite loop.");
  loading_ = true;
  cost_.reset(new SmoothMinimum());
  for (const std::string& comonent_name : get_component_names()) {
    auto* component = node()->app()->findComponentByName<PlannerCostBuilder>(comonent_name);
    if (component == nullptr) {
      LOG_ERROR("Failed to load the component: %s", comonent_name.c_str());
      continue;
    }
    cost_->addCost(component->build());
    builders_.push_back(component);
  }
  loading_ = false;
  return static_cast<PlannerCost*>(cost_.get());
}

void SmoothMinimumBuilder::update(double start_time, double end_time) {
  cost_->setMinimumSmoothing(get_minimum_smoothing());
  for (auto* builder : builders_) {
    builder->update(start_time, end_time);
  }
}

void SmoothMinimumBuilder::destroy() {
  cost_.reset();
  for (auto* builder : builders_) {
    builder->destroy();
  }
  builders_.clear();
}

PlannerCost* SmoothMinimumBuilder::get() {
  return static_cast<PlannerCost*>(cost_.get());
}

}  // namespace planner_cost
}  // namespace isaac
