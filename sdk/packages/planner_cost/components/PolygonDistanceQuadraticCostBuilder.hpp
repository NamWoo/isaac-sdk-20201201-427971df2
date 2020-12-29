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

#include "engine/alice/alice_codelet.hpp"
#include "engine/gems/geometry/polyline.hpp"
#include "packages/planner_cost/components/PlannerCostBuilder.hpp"
#include "packages/planner_cost/gems/polygon_distance_quadratic_cost.hpp"

namespace isaac {
namespace planner_cost {

// Implementation of PlannerCostBuilder that uses the PolygonDistanceQuadraticCost evaluation.
// It takes a polygon as a config parameter and will produces a smooth quadratic error based
// on the distance to the polygon. If the polygon is provided in clockwise order, then the cost will
// penalize being outside the polygon, otherwise it will penalize being inside.
class PolygonDistanceQuadraticCostBuilder : public PlannerCostBuilder {
 public:
  PlannerCost* build() override;
  void update(double start_time, double end_time) override;
  void destroy() override;
  PlannerCost* get() override;

  // Gain of the quadratic cost: 0.5 * gain * min(0, offset + distance)^2
  ISAAC_PARAM(double, gain, 1.0);
  // offset of the quadratic cost: 0.5 * gain * min(0, offset + distance)^2
  ISAAC_PARAM(double, offset, 0.0);
  // Index for [pos_x, pos_y] inside state
  ISAAC_PARAM(Vector2i, indices, Vector2i(0, 1));
  // Parameter to track if the polygon needs to be updated.
  ISAAC_PARAM(bool, update_polygon, true);
  // Takes list of 2d points the represents the polygon. If the vertices are provided in clock-wise
  // order, then it penalizes for being outside the polygon.
  ISAAC_PARAM(geometry::Polyline2d, polygon, {});

 private:
  // This is an implementation of the update function, it does not require the time interval as it
  // only gets the target polyline from message and passes it along to the cost_ function.
  void updateImpl();

  std::unique_ptr<PolygonDistanceQuadraticCost> cost_;
};

}  // namespace planner_cost
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner_cost::PolygonDistanceQuadraticCostBuilder);
