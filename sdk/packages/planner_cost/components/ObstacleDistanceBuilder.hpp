/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "packages/map/ObstacleAtlas.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/obstacle_distance.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder.
// This component takes an obstacle name from the config and build a ObstacleDistance function.
// It also needs to know the expected frame of the state in order to prepare the transformation from
// the state frame to the obstacle frame.
class ObstacleDistanceBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Name of the obstacle in the Atlas
  ISAAC_PARAM(std::string, obstacle_name);

  // Name of frame used by the cost function
  ISAAC_PARAM(std::string, reference_frame, "odom");

 private:
  std::unique_ptr<ObstacleDistance> obstacle_distance_;
  map::ObstacleAtlas* obstacle_atlas_;
  std::shared_ptr<const map::ObstacleAtlas::Obstacle> obstacle_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::ObstacleDistanceBuilder);
