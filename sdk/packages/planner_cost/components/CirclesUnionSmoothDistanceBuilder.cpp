/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/CirclesUnionSmoothDistanceBuilder.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* CirclesUnionSmoothDistanceBuilder::build() {
  robot_shape_ = node()->app()->findComponent<planner::SphericalRobotShapeComponent>();
  ASSERT(robot_shape_ != nullptr,
         "Could not find obstacle atlas, which is needed for obstacle avoidance. Either set "
         "obstacle_names parameter empty or create a node with a map::ObstacleAtlas component.");

  distance_builder_ = node()->app()->findComponentByName<PlannerCostBuilder>(get_component_name());
  ASSERT(distance_builder_ != nullptr,
         "Failed to load the component: %s", get_component_name().c_str());
  smooth_distance_.reset(new CirclesUnionSmoothDistance());
  smooth_distance_->setDistanceFunction(distance_builder_->build());
  return static_cast<PlannerCost*>(smooth_distance_.get());
}

void CirclesUnionSmoothDistanceBuilder::update(double start_time, double end_time) {
  distance_builder_->update(start_time, end_time);
  smooth_distance_->setCircles(robot_shape_->get_circles());
  smooth_distance_->setIndices(get_indices_mapping());
}

void CirclesUnionSmoothDistanceBuilder::destroy() {
  smooth_distance_.reset();
  distance_builder_->destroy();
}

PlannerCost* CirclesUnionSmoothDistanceBuilder::get() {
  return static_cast<PlannerCost*>(smooth_distance_.get());
}

}  // namespace planner_cost
}  // namespace isaac
