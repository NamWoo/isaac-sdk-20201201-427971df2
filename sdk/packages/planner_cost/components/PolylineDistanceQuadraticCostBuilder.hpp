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
#include "engine/gems/geometry/polyline.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/polyline_distance_quadratic_cost.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the PolylineDistanceQuadraticCost evaluation.
// It takes a path to follow as a config parameter and will produces a smooth quadratic error based
// on the distance to the polyline.
// If speed_reward is not 0.0, an additional linear reward will be added on the speed to have the
// robot moving in the direction of the path.
class PolylineDistanceQuadraticCostBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Gain of the quadratic cost: 0.5 * gain * distance^2
  ISAAC_PARAM(double, gain);
  // Index for [pos_x, pos_y, speed] inside state.
  ISAAC_PARAM(Vector3i, indices, Vector3i(0, 1, 3));
  // Parameter to track if the polyline needs to be updated.
  ISAAC_PARAM(bool, update_polyline, true);
  // Speed reward to make the robot move in the direction of the polyline. (Positive reward means
  // moving in the direction of the trajectory while a negative reward will reward moving in the
  // opposite direction).
  ISAAC_PARAM(double, speed_reward, 0.0);
  // The speed reward will only be applied if the distance to the end target is more than this
  // distance (this effectively set the speed_reward to 0.0 when the robot is getting too close)
  ISAAC_PARAM(double, min_distance_for_reward, 0.0);
  // Takes list of 2d points as the polyline
  ISAAC_PARAM(geometry::Polyline2d, polyline, {});
  // The maximum number of waypoints to be forwarded to the planner_cost.
  // Computing the smooth distance to a polyline is expensive, reducing the number of waypoints can
  // significantly improve the performance.
  ISAAC_PARAM(int, maximum_number_points, 20);
  // If two waypoints are closer than this distance, the later will be ignored (unless it is the
  // last one). Computing the distance to a polyline is expensive, and waypoint close to each other
  // do not bring much information.
  ISAAC_PARAM(double, minimum_distance_waypoint, 1.0);
  // The name of the frame in which the polyline's points are provided.
  ISAAC_PARAM(std::string, polyline_frame, "odom");
  // The name of the frame used for planning.
  ISAAC_PARAM(std::string, planning_frame, "odom");
  // The name of the robot frame.
  ISAAC_PARAM(std::string, robot_frame, "robot");

 private:
  // This is an implementation of the update function, it does not require the time interval as it
  // only gets the target polyline from message and passes it along to the cost_ function.
  void updateImpl(double time);

  std::unique_ptr<PolylineDistanceQuadraticCost> cost_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::PolylineDistanceQuadraticCostBuilder);
