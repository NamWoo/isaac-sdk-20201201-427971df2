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
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/distance_quadratic_cost.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that use the DistanceQuadraticCost evaluation.
// It loads the distance function implementation from a component specified from the config
// parameter `component_name`.
// The list of cost function is provided using the config parameter costs. See the documentation
// of the parameter for more details.
class DistanceQuadraticCostBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Names of the component Implementating a PlannerCostBuilder to be used as distance function
  ISAAC_PARAM(std::string, component_name);
  // The indices of the [pos_x, pos_y, heading, linear_speed] inside state.
  ISAAC_PARAM(Vector4i, indices_mapping, Vector4i(0, 1, 2, 3));
  // List of cost to add based on the distance function.
  // Each cost is of the following form:
  //   0.5 * gain * min(0, distance - speed * speed_gradient - target)^2
  // where Vector3d is used to represent the triplet: [gain, target, speed_gradient]
  ISAAC_PARAM(std::vector<Vector3d>, costs,
              std::vector<Vector3d>({Vector3d(1.0, 0.25, 1.0), Vector3d(500.0, 0.1, 0.0)}));

 private:
  std::unique_ptr<DistanceQuadraticCost> distance_quadratic_cost_;
  PlannerCostBuilder* builder_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::DistanceQuadraticCostBuilder);
