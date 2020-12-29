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
#include "packages/planner/SphericalRobotShapeComponent.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/circles_union_smooth_distance.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that use the CirclesUnionSmoothDistance evaluation.
// This component loads the SphericalRobotShapeComponent to get the list of circles that represents
// the robot and loads a distance function from another component (specified by the config
// parameter `component_name`).
class CirclesUnionSmoothDistanceBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Names of the component Implementating a PlannerCostBuilder to be used as distance function
  ISAAC_PARAM(std::string, component_name);

  // The indices of the [pos_x, pos_y, heading] inside state.
  ISAAC_PARAM(Vector3i, indices_mapping, Vector3i(0, 1, 2));

 private:
  std::unique_ptr<CirclesUnionSmoothDistance> smooth_distance_;
  // Component holding the distance function used.
  PlannerCostBuilder* distance_builder_;
  // RobotShape that contains the list of circles
  planner::SphericalRobotShapeComponent* robot_shape_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::CirclesUnionSmoothDistanceBuilder);
