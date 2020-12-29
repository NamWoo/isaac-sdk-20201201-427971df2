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

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/n_sphere.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"
#include "packages/planner_cost/gems/pose2_transform.hpp"
#include "packages/planner_cost/gems/scalar_addition.hpp"
#include "packages/planner_cost/gems/smooth_minimum.hpp"

namespace isaac {
namespace planner_cost {

// This is an implementation of PlannerCost.
// It takes a list of circles in the output frame of the state (for example, if state correspond to
// the position of the robot, then the circles are in robot frame), it also takes a distance
// function to be called for each circle and it computes the smooth minimum accross all the circles.
// To simplify the math it creates a SmoothMinimum object as well as one ScalarAddition (to
// substract the radius) and one Pose2Transform (to move the circle in reference frame) per circle.
// By default it assumes that the position (x, y, heading) corresponds to the first three
// dimensions of the state vector. If this is not the case you can set the position of (x, y, h)
// using setIndices.
class CirclesUnionSmoothDistance : public PlannerCost {
 public:
  CirclesUnionSmoothDistance() = default;

  bool isValid(double time, const VectorXd& state) override;
  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets a PlannerCost to be called for each circle
  void setDistanceFunction(PlannerCost* distance_function) {
    distance_function_ = distance_function;
  }

  // Sets the indices of the [pos_x, pos_y, heading] inside state.
  void setIndices(const Vector3i& indices) {
    indices_ = indices;
    for (auto& pose2_transform : pose2_transforms_) {
      pose2_transform.setIndices(indices);
    }
  }

  // Creates a ScalarAddition and Pose2Transform for each circle. The distance_function will be:
  //  smooth_min(ScalarAddition(Pose2Transform(distance_function_) - center.radius))
  void setCircles(const std::vector<geometry::CircleD>& circles);

  // Sets the smoothing factor.
  void setMinimumSmoothing(double minimum_smoothing) {
    minimum_smoothing_ = minimum_smoothing;
    smooth_minimum_.setMinimumSmoothing(minimum_smoothing_);
  }

 private:
  PlannerCost* distance_function_ = nullptr;
  // Indices of pos_x, pos_y, heading.
  Vector3i indices_ = Vector3i(0, 1, 2);
  // Used to compute the smooth minimum.
  SmoothMinimum smooth_minimum_;
  // Default value that does not create numerical instability and provide good result in general
  // cases.
  double minimum_smoothing_ = 20.0;
  // One per circle, it is used to substract the radius
  std::vector<ScalarAddition> scalar_additions_;
  std::vector<Pose2Transform> pose2_transforms_;
};

}  // namespace planner_cost
}  // namespace isaac
