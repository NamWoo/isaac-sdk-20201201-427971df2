/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/PolygonSpeedLimitBuilder.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "messages/math.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* PolygonSpeedLimitBuilder::build() {
  // Get the map:
  polygon_layer_ =
      node()->app()->findComponentByName<map::PolygonMapLayer>(get_polygon_layer_name());
  if (polygon_layer_ == nullptr) {
    LOG_ERROR("Failed to load the polygon layer: %s", get_polygon_layer_name().c_str());
    return nullptr;
  }
  cost_.reset(new PolygonSpeedLimit());
  updateImpl(0.0);
  return static_cast<PlannerCost*>(cost_.get());
}

void PolygonSpeedLimitBuilder::update(double start_time, double /*end_time*/) {
  updateImpl(start_time);
}

void PolygonSpeedLimitBuilder::destroy() {
  cost_.reset();
}

PlannerCost* PolygonSpeedLimitBuilder::get() {
  return static_cast<PlannerCost*>(cost_.get());
}

void PolygonSpeedLimitBuilder::updateImpl(double time) {
  cost_->setGain(get_gain());
  cost_->setSpeedLimit(get_speed_limit());
  cost_->setIndices(get_indices());
  cost_->setSigma(get_sigma());
  cost_->setForcePositiveDefiniteHessian(get_force_positive_definite_hessian());
  // Get the transformation
  const auto maybe_planning_T_polygon =
      node()->pose().tryGetPose2XY(get_planning_frame(), get_polygon_frame(), time);
  if (maybe_planning_T_polygon == std::nullopt) return;
  std::vector<geometry::Polygon2D> polygons;
  auto maybe_polygon_names = try_get_polygon_names();
  if (maybe_polygon_names == std::nullopt) {
    for (const auto& polygon : polygon_layer_->polygons()) {
      polygons.push_back(polygon.second);
    }
  } else {
    for (const std::string& name : *maybe_polygon_names) {
      auto maybe_polygon = polygon_layer_->findByName(name);
      if (maybe_polygon != std::nullopt) {
        polygons.push_back(*maybe_polygon);
      }
    }
  }

  for (auto& polygon : polygons) {
    for (Vector2d& point : polygon.points) {
      point = *maybe_planning_T_polygon * point;
    }
  }
  cost_->setPolygons(std::move(polygons));
}

}  // namespace planner_cost
}  // namespace isaac
