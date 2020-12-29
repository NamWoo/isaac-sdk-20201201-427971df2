/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/PolygonDistanceQuadraticCostBuilder.hpp"

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "messages/math.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* PolygonDistanceQuadraticCostBuilder::build() {
  cost_.reset(new PolygonDistanceQuadraticCost());
  // Make sure we are updating the polygon the first time.
  set_update_polygon(true);
  updateImpl();
  return static_cast<PlannerCost*>(cost_.get());
}

void PolygonDistanceQuadraticCostBuilder::update(double /*start_time*/, double /*end_time*/) {
  updateImpl();
}

void PolygonDistanceQuadraticCostBuilder::destroy() {
  cost_.reset();
}

PlannerCost* PolygonDistanceQuadraticCostBuilder::get() {
  return static_cast<PlannerCost*>(cost_.get());
}

void PolygonDistanceQuadraticCostBuilder::updateImpl() {
  cost_->setGain(get_gain());
  cost_->setOffset(get_offset());
  cost_->setIndices(get_indices());
  if (!get_update_polygon()) return;
  set_update_polygon(false);
  cost_->setPolygon(geometry::Polygon2D{get_polygon()});
}

}  // namespace planner_cost
}  // namespace isaac
