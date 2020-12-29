/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/types.hpp"
#include "packages/map/gems/obstacle.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// This is an implementation of PlannerCost.
// It takes a map::Obstacle implementation and uses it to compute a distance function.
// It computes the distance from a single point to the obstacle, it can be used with
// CirclesUnionSmoothDistance if we want to compute the distance from a SphericalRobotModel to the
// obstacle.
// By default it assumes that the position (x, y) corresponds to the first two dimensions of the
// state vector. If this is not the case you can set the position of (x, y) using setIndices.
class ObstacleDistance : public PlannerCost {
 public:
  ObstacleDistance() = default;

  bool isValid(double time, const VectorXd& state) override;
  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the obstacle.
  void setObstacle(map::Obstacle* obstacle) {
    obstacle_ = obstacle;
  }

  // Sets the indices of the (x, y) coordinates.
  void setIndices(const Vector2i& indices) {
    indices_ = indices;
  }

  // Sets the transformation from the reference frame of the state to the obstacle frame.
  void setObstacleTStateFrame(const Pose2d& obstacle_T_state_frame) {
    obstacle_T_state_frame_ = obstacle_T_state_frame;
  }

 private:
  // Obstacle used to compute the distance.
  map::Obstacle* obstacle_ = nullptr;
  // Indices of pos_x, pos_y.
  Vector2i indices_ = Vector2i(0, 1);
  // Constant transformation to transform from the reference frame of the state to the frame of the
  // obstacle.
  Pose2d obstacle_T_state_frame_;
};

}  // namespace planner_cost
}  // namespace isaac
