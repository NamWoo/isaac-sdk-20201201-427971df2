/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/DotProductCostBuilder.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* DotProductCostBuilder::build() {
  cost_.reset(new DotProductCost());
  updateImpl();
  return static_cast<PlannerCost*>(cost_.get());
}

void DotProductCostBuilder::update(double /*start_time*/, double /*end_time*/) {
  updateImpl();
}

void DotProductCostBuilder::destroy() {
  cost_.reset();
}

PlannerCost* DotProductCostBuilder::get() {
  return static_cast<PlannerCost*>(cost_.get());
}

void DotProductCostBuilder::updateImpl() {
  cost_->setOffset(get_offset());
  cost_->setDotVector(get_dot_vector());
}

}  // namespace planner_cost
}  // namespace isaac
