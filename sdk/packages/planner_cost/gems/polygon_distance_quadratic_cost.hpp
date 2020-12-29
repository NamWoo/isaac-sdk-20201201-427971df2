/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <utility>

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/polygon.hpp"
#include "engine/gems/geometry/polyline.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// This is an implementation of PlannerCost.
// It takes a polygon and computes a smooth distance from a point to the edge of the polygons, then
// it computes a quadratic cost from it: cost = 0.5 * gain * max(0, dist + offset)^2.
// The polygon needs to be simple (no self intersection). The distance is negative inside the
// polygon and positive outside if and only if the polygon vertices are provided in clockwise order.
// If you want to penalize being outside the polygon, provide the vertices in clock-wise order, and
// if you want to penalize being inside the polygon, provide the vertices in anti-clockwise order.
// By default it is assumed that the position (x, y) corresponds to the first and second dimension
// of the state vector. If this is not the case you can set the position of (x, y) using setIndices.
class PolygonDistanceQuadraticCost : public PlannerCost {
 public:
  PolygonDistanceQuadraticCost() = default;

  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  // Currently the hessian is approximated by assuming the hessian of the smooth distance is null.
  // This approximation makes it more stable with LQR which does not like non positive definite
  // hessian. As a result only gradient * gradient.transpose() contribute to the hessian, which is
  // coming from the quadratic error.
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the polygon to compute the smooth distance from. If provided in clockwise order, the
  // distance will be positive outside the polygon, otherwise the distance will be positive inside.
  void setPolygon(geometry::Polygon2D polygon) {
    polyline_ = std::move(polygon.points);
    // Duplicate the first vertex to close the polyline.
    polyline_.push_back(polyline_.front());
  }

  // Sets the gain associated to the quadratic cost
  void setGain(double gain) {
    gain_ = gain;
  }

  // Sets the offset used to compute the final distance.
  void setOffset(double offset) {
    offset_ = offset;
  }

  // Sets the indices of the [pos_x, pos_y] inside state.
  void setIndices(const Vector2i& indices) {
    indices_ = indices;
  }

 private:
  // Indices of pos_x, pos_y, heading.
  Vector2i indices_ = Vector2i(0, 1);
  // Polyline to compute the distance from. The first and last point are identical, the polyline
  // represents a polygon.
  geometry::Polyline2d polyline_;
  // Gain assigned to the quadratic cost.
  double gain_ = 1.0;
  // Offet to be added to the distance.
  double offset_ = 0.0;
};

}  // namespace planner_cost
}  // namespace isaac
