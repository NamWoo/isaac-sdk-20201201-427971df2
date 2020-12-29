/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/components/PolylineDistanceQuadraticCostBuilder.hpp"

#include <string>
#include <utility>

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "messages/math.hpp"

namespace isaac {
namespace planner_cost {

PlannerCost* PolylineDistanceQuadraticCostBuilder::build() {
  cost_.reset(new PolylineDistanceQuadraticCost());
  // Make sure we are updating the polyline the first time.
  set_update_polyline(true);
  updateImpl(node()->clock()->time());
  return static_cast<PlannerCost*>(cost_.get());
}

void PolylineDistanceQuadraticCostBuilder::update(double start_time, double /*end_time*/) {
  updateImpl(start_time);
}

void PolylineDistanceQuadraticCostBuilder::destroy() {
  cost_.reset();
}

PlannerCost* PolylineDistanceQuadraticCostBuilder::get() {
  return static_cast<PlannerCost*>(cost_.get());
}

void PolylineDistanceQuadraticCostBuilder::updateImpl(double time) {
  cost_->setGain(get_gain());
  cost_->setIndices(get_indices());
  const std::string planning_frame = get_planning_frame();
  if (get_update_polyline()) {
    geometry::Polyline2d polyline = get_polyline();
    if (polyline.empty()) {
      set_update_polyline(false);
      cost_->setPolyline({});
      // If the polyline is empty, we cannot find the direction of the motion.
      cost_->setSpeedReward(0.0);
      return;
    }
    const std::string polyline_frame = get_polyline_frame();
    // Transform the polyline in the right coordinate frame.
    if (planning_frame != polyline_frame) {
      const auto maybe_planning_T_polyline =
          node()->pose().tryGetPose2XY(planning_frame, polyline_frame, time);
      if (!maybe_planning_T_polyline) return;
      for (Vector2d& waypoint : polyline) {
        waypoint = *maybe_planning_T_polyline * waypoint;
      }
    }
    // Reduce the size of the polyline to reduce the compute time of the distance to the polyline.
    const size_t maximum_number_points = get_maximum_number_points();
    const double minimum_distance_waypoint = get_minimum_distance_waypoint();
    const double minimum_squared_distance_waypoint =
        minimum_distance_waypoint * minimum_distance_waypoint;
    geometry::Polyline2d shorter_polyline = {polyline.front()};
    for (size_t idx = 0; idx < polyline.size() && shorter_polyline.size() < maximum_number_points;
         idx++) {
      // If the waypoint is far enough or if it is the last waypoint, we add it to the list.
      if ((polyline[idx] - shorter_polyline.back()).squaredNorm() >=
          minimum_squared_distance_waypoint || idx + 1 == polyline.size()) {
        shorter_polyline.push_back(polyline[idx]);
      }
    }
    cost_->setPolyline(std::move(shorter_polyline));
    set_update_polyline(false);
  }
  // Set up the speed reward if needed
  // If the polyline is empty, we can return right away.
  const auto& polyline = cost_->getPolyline();
  if (polyline.empty()) return;
  // If the reward is 0.0, we can also return.
  const double reward = get_speed_reward();
  if (reward == 0.0) return;
  // Get the position of the robot, and returns in case of failrue
  const auto maybe_planning_T_robot =
      node()->pose().tryGetPose2XY(planning_frame, get_robot_frame(), time);
  if (!maybe_planning_T_robot) return;
  // Check if we are close enough
  const Vector2d delta_position = polyline.back() - maybe_planning_T_robot->translation;
  const Vector2d path_direction =
      polyline.size() > 1 ? polyline[1] - polyline[0] : delta_position;
  polyline.back() - maybe_planning_T_robot->translation;
  if (delta_position.squaredNorm() < Square(get_min_distance_for_reward())) return;
  const Vector2d car_direction = maybe_planning_T_robot->rotation.asDirection();
  const bool forward = path_direction.dot(car_direction) >= 0.0;
  // For a forward motion, we need a negative cost (as the lqr solver is trying to minimize the
  // score).
  cost_->setSpeedReward(forward ? -reward : reward);
}

}  // namespace planner_cost
}  // namespace isaac
