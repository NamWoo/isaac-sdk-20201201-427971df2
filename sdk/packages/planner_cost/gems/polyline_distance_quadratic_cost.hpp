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
#include "engine/gems/geometry/polyline.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// This is an implementation of PlannerCost.
// It takes a polyline and computes a smooth distance from a point to a polyline, then it computes
// a quadratic cost from it: cost = 0.5 * gain * dist * dist.
// In addition if a speed reward is provided, a linear cost will be added.
// By default it is assumed that the position (x, y) corresponds to the first and second dimension
// of the state vector, while the speed is the fourth. If this is not the case you can set the
// correct indices of (x, y, speed) using setIndices.
class PolylineDistanceQuadraticCost : public PlannerCost {
 public:
  PolylineDistanceQuadraticCost() = default;

  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  // Currently the hessian is approximated by assuming the hessian of the smooth distance is null.
  // This approximation makes it more stable with LQR which does not like non positive definite
  // hessian. As a result only gradient * gradient.transpose() contribute to the hessian, which is
  // coming from the quadratic error.
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the polyline to compute the smooth distance from.
  void setPolyline(geometry::Polyline2d polyline) {
    polyline_ = std::move(polyline);
  }
  // Returns a reference to the polyline.
  const geometry::Polyline2d& getPolyline() const { return polyline_;}

  // Sets the gain associated to the quadratic cost
  void setGain(double gain) {
    gain_ = gain;
  }

  // Sets the reward associated to the speed in order to have the robot moving.
  void setSpeedReward(double speed_reward) {
    speed_reward_ = speed_reward;
  }

  // Sets the indices of the [pos_x, pos_y, speed] inside state.
  void setIndices(const Vector3i& indices) {
    indices_ = indices;
  }

 private:
  // Indices of pos_x, pos_y, and linear speed.
  Vector3i indices_ = Vector3i(0, 1, 3);
  // Polyline to compute the distance from.
  geometry::Polyline2d polyline_;
  // Gain assigned to the quadratic cost.
  double gain_ = 1.0;
  // Gain assigned to the quadratic cost.
  double speed_reward_ = 0.0;
};

}  // namespace planner_cost
}  // namespace isaac
