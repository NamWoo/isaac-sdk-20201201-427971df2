/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/ObstacleDistanceBuilder.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* ObstacleDistanceBuilder::build() {
  obstacle_atlas_ = node()->app()->findComponent<map::ObstacleAtlas>();
  obstacle_distance_.reset(new ObstacleDistance());
  ASSERT(obstacle_atlas_ != nullptr,
         "Could not find obstacle atlas, which is needed for obstacle avoidance. Either set "
         "obstacle_names parameter empty or create a node with a map::ObstacleAtlas component.");
  return static_cast<PlannerCost*>(obstacle_distance_.get());
}

void ObstacleDistanceBuilder::update(double start_time, double end_time) {
  obstacle_ = obstacle_atlas_->tryGetObstacle(get_obstacle_name());
  if (obstacle_.get() == nullptr) {
    obstacle_distance_->setObstacle(nullptr);
    return;
  }
  const auto maybe_pose = node()->pose().tryGetPose2XY(
      obstacle_->frame, get_reference_frame(), ToSeconds(obstacle_->timestamp));
  if (!maybe_pose) {
    obstacle_distance_->setObstacle(nullptr);
    obstacle_ = nullptr;
    return;
  }
  obstacle_distance_->setObstacle(obstacle_->obstacle.get());
  obstacle_distance_->setObstacleTStateFrame(*maybe_pose);
}

void ObstacleDistanceBuilder::destroy() {
  obstacle_distance_.reset();
  obstacle_ = nullptr;
}

PlannerCost* ObstacleDistanceBuilder::get() {
  return static_cast<PlannerCost*>(obstacle_distance_.get());
}

}  // namespace planner_cost
}  // namespace isaac
