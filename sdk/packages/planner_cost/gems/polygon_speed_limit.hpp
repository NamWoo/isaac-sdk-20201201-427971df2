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
#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/polygon.hpp"
#include "engine/gems/geometry/polyline.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// This is an implementation of PlannerCost.
// It takes a list of disjoint polygons and computes a smooth quadratic penality based on the speed
// and whether we are inside one of the polygons. The penalty is set to be 0 if the speed is below
// the speed limit or if we are outside the polygons and it gradually increases to be:
//   0.5 * gain * max(0, speed - speed_limit)^2
// The deeper we are inside one of the polygon, the better the approximation is:
//   0.5 * gain * max(0, speed-speed_limit)^2 * distance^2 / (distance^2 + sigma^2)
// The sigma parameter controls how fast we move from a penalty of 0 to the quadratic penalty, at
// a distance sigma we are at 50% of the quadratic penalty.
class PolygonSpeedLimit : public PlannerCost {
 public:
  PolygonSpeedLimit() = default;

  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  // Currently the hessian is approximated by assuming the hessian of the smooth distance is null.
  // This approximation makes it more stable with LQR which does not like non positive definite
  // hessian.
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the list polygons to compute the smooth distance from.
  void setPolygons(std::vector<geometry::Polygon2D> polygons) {
    polygons_ = std::move(polygons);
    for (auto& polygon : polygons_) {
      ASSERT(static_cast<int>(polygon.points.size()) >= 3,
             "The polygon needs at least 3 points to be valid, only %zu were provided",
             polygon.points.size());
      if (!IsAlmostZero((polygon.points.front() - polygon.points.back()).squaredNorm())) {
        // Duplicate the first vertex to close the polyline.
        polygon.points.push_back(polygon.points.front());
      }
    }
  }

  // Sets the sigma value (see description above to see how it is used in the formula)
  void setSigma(double sigma) {
    ASSERT(!IsAlmostZero(sigma), "sigma cannot be 0.0");
    squared_sigma_ = sigma * sigma;
  }

  // Sets the gain associated to the quadratic cost
  void setGain(double gain) {
    gain_ = gain;
  }

  // Sets the speed_limit associated to this area
  void setSpeedLimit(double speed_limit) {
    ASSERT(speed_limit >= 0.0, "speed_limit must be positive (%lf)", speed_limit);
    speed_limit_ = speed_limit;
  }

  // Sets the indices of the [pos_x, pos_y, linear_speed] inside state.
  void setIndices(const Vector3i& indices) {
    indices_ = indices;
  }

  // Whether or not we force the hessian to be positive definite. For optimization problem it might
  // be important for the hessian to be positive definite. If this parameter is set to true, we will
  // approximate the hessian by a form which is positive definite.
  void setForcePositiveDefiniteHessian(bool force_positive_definite_hessian) {
    force_positive_definite_hessian_ = force_positive_definite_hessian;
  }

 private:
  // Indices of pos_x, pos_y, heading.
  Vector3i indices_ = Vector3i(0, 1, 3);
  // Polyline to compute the distance from. The first and last point are identical, the polyline
  // represents a polygon.
  std::vector<geometry::Polygon2D> polygons_;
  // Gain assigned to the quadratic cost.
  double gain_ = 1.0;
  // Speed limit.
  double speed_limit_ = 1.0;
  // Squared value of sigma cached. It controls how smooth is the transition to be penalize when
  // crossing the polygon's boudaries.
  double squared_sigma_ = 0.1;
  // Force the hessian to be definite positive to prevent the solver to fail.
  bool force_positive_definite_hessian_ = true;
};

}  // namespace planner_cost
}  // namespace isaac
