/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/SmoothLinearTargetCostBuilder.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* SmoothLinearTargetCostBuilder::build() {
  cost_.reset(new SmoothLinearTargetCost());
  updateImpl();
  return static_cast<PlannerCost*>(cost_.get());
}

void SmoothLinearTargetCostBuilder::update(double /*start_time*/, double /*end_time*/) {
  updateImpl();
}

void SmoothLinearTargetCostBuilder::destroy() {
  cost_.reset();
}

PlannerCost* SmoothLinearTargetCostBuilder::get() {
  return static_cast<PlannerCost*>(cost_.get());
}

void SmoothLinearTargetCostBuilder::updateImpl() {
  cost_->setScaler(try_get_scaler());
  cost_->setTarget(get_target());
  cost_->setGain(get_gain());
  cost_->setEpsilon(get_epsilon());
  cost_->setForcePositiveDefiniteHessian(get_force_positive_definite_hessian());
}

}  // namespace planner_cost
}  // namespace isaac
