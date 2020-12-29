/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/RangeConstraintsCostBuilder.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* RangeConstraintsCostBuilder::build() {
  cost_.reset(new RangeConstraintsCost());
  updateImpl();
  return static_cast<PlannerCost*>(cost_.get());
}

void RangeConstraintsCostBuilder::update(double /*start_time*/, double /*end_time*/) {
  updateImpl();
}

void RangeConstraintsCostBuilder::destroy() {
  cost_.reset();
}

PlannerCost* RangeConstraintsCostBuilder::get() {
  return static_cast<PlannerCost*>(cost_.get());
}

void RangeConstraintsCostBuilder::updateImpl() {
  cost_->setGains(get_gains());
  cost_->setMinValue(get_min_value());
  cost_->setMaxValue(get_max_value());
  // Check the parameters are consistent.
  ASSERT(cost_->gains().size() == cost_->min_value().size(),
         "The number of dimension of the gains (%d) does not match the dimension of min_value (%d)",
         cost_->gains().size(), cost_->min_value().size());
  ASSERT(cost_->gains().size() == cost_->max_value().size(),
         "The number of dimension of the gains (%d) does not match the dimension of max_value (%d)",
         cost_->gains().size(), cost_->max_value().size());
}

}  // namespace planner_cost
}  // namespace isaac
