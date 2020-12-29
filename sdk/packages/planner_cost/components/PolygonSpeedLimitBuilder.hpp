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
#include "engine/gems/geometry/polyline.hpp"
#include "packages/map/PolygonMapLayer.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/polygon_speed_limit.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the PolygonSpeedLimit evaluation.
// It loads a list of polygons from PolygonMapLayer (either by name or all of them). and penalizes
// if the robot exceeds the speed limit inside one of the polygons.
// The cost used will quickly approach a quadratic cost but has a smooth border that can be
// configured using the sigma parameter. The final cost is:
// 0.5 * gain * max(0, speed - speed_limit)^2 * distance^2 / (distance^2 + sigma^2).
// where distance is the distance inside one of the polygons and speed_limit and sigma are defined
// by the config parameters.
class PolygonSpeedLimitBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Gain of the cost (see above for the exact formula)
  ISAAC_PARAM(double, gain, 1.0);
  // Maximum allowed speed. If the robot is below this limit, the cost is always 0, otherwise if the
  // robot is inside one of the polygons, it is penalized with an almost quadratic cost (see the
  // formula above).
  ISAAC_PARAM(double, speed_limit, 0.5);
  // Index for [pos_x, pos_y, speed] inside state
  ISAAC_PARAM(Vector3i, indices, Vector3i(0, 1, 3));
  // Controls the spread where the function behave as a quadratic function. (see the formula above).
  // Sigma needs to be different from 0.0.
  ISAAC_PARAM(double, sigma, 0.1);
  // Whether or not we force the hessian to be positive definite. For optimization problem it might
  // be important for the hessian to be positive definite. If this parameter is set to true, we will
  // approximate the hessian by a form which is positive definite.
  ISAAC_PARAM(bool, force_positive_definite_hessian, true);
  // Name of the component that contains the PolygonMapLayer used to get the list of polygons;
  ISAAC_PARAM(std::string, polygon_layer_name);
  // List of the names of the polygon in the PolygonMapLayer to be used to compute the cost.
  // If it is not provided all the polygons  will be used.
  ISAAC_PARAM(std::vector<std::string>, polygon_names);

  // The name of the frame in which the polygon's points are provided.
  ISAAC_PARAM(std::string, polygon_frame, "world");
  // The name of the frame used for planning.
  ISAAC_PARAM(std::string, planning_frame, "odom");

 private:
  // This is an implementation of the update function, it does not require the time interval as it
  // only gets the target polyline from message and passes it along to the cost_ function.
  void updateImpl(double time);

  std::unique_ptr<PolygonSpeedLimit> cost_;

  map::PolygonMapLayer* polygon_layer_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::PolygonSpeedLimitBuilder);
