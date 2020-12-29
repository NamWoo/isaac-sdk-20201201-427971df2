/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// This is an implementation of PlannerCost.
// It takes another PlannerCost as input as well as the a Pose2 transformation and computes:
//   evaluate(state) = cost.evaluate(Pose2From(state) * transform);
// where Pose2From(state) is extracting a Pose2d from the state using indices_.
// By default it assumes that the position (x, y, heading) corresponds to the first three
// dimensions of the state vector. If this is not the case you can set the position of (x, y, h)
// using setIndices.
// This cost function is calling another PlannerCost for a constant position set in the coordinate
// frame set by the vector state, for example:
// If the state describes the position of the robot, and transform is describing the position of an
// object attached to the robot such as a lidar (transform_ = robot_T_lidar), then this PlannerCost
// evaluate the provided cost function at the position of the lidar.
class Pose2Transform : public PlannerCost {
 public:
  Pose2Transform() = default;

  bool isValid(double time, const VectorXd& state) override;
  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets the PlannerCost to be called after the state transformed.
  void setCost(PlannerCost* cost) {
    cost_ = cost;
  }

  // Sets the indices of the (x, y, heading) coordinates
  void setIndices(const Vector3i& indices) {
    indices_ = indices;
  }

  // Sets the transformation to use.
  void setPose(const Pose2d& transform) {
    transform_ = transform;
  }

 private:
  // Helper function that populates state_ and state_as_pose_.
  void populateState(const VectorXd& state);

  PlannerCost* cost_ = nullptr;
  // Indices of pos_x, pos_y, heading.
  Vector3i indices_ = Vector3i(0, 1, 2);
  // transformation to be applied.
  Pose2d transform_;
  // Correspond to the new state as a Pose2d.
  Pose2d state_as_pose_;
  // The transformed state that will be used to call cost_
  Vector3d state_;
  // gradient_ and hessian_ are here to avoid any memory allocation
  Vector3d gradient_;
  Matrix3d hessian_;
};

}  // namespace planner_cost
}  // namespace isaac
