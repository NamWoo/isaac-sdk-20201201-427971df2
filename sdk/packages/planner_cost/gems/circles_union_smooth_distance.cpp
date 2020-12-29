/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/circles_union_smooth_distance.hpp"

#include <vector>

#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

bool CirclesUnionSmoothDistance::isValid(double time, const VectorXd& state) {
  return distance_function_ == nullptr || distance_function_->isValid(time, state);
}

double CirclesUnionSmoothDistance::evaluate(double time, const VectorXd& state) {
  if (distance_function_ == nullptr) return 0.0;
  return smooth_minimum_.evaluate(time, state);
}

void CirclesUnionSmoothDistance::addGradient(double time, const VectorXd& state,
                                 Eigen::Ref<VectorXd> gradient) {
  if (distance_function_ == nullptr) return;
  // TODO(ben): Consider only using a Vector3d for the gradient
  smooth_minimum_.addGradient(time, state, gradient);
}

void CirclesUnionSmoothDistance::addHessian(double time, const VectorXd& state,
                                Eigen::Ref<MatrixXd> hessian) {
  if (distance_function_ == nullptr) return;
  // TODO(ben): Consider only using a Matrix3d for the hessian
  smooth_minimum_.addHessian(time, state, hessian);
}

void CirclesUnionSmoothDistance::setCircles(const std::vector<geometry::CircleD>& circles) {
  // Reset smooth_minimum_, otherwise we can reuse the existing one
  if (pose2_transforms_.size() != circles.size()) {
    smooth_minimum_ = SmoothMinimum();
    smooth_minimum_.setMinimumSmoothing(minimum_smoothing_);
    pose2_transforms_.resize(circles.size());
    scalar_additions_.resize(circles.size());
    for (size_t idx = 0; idx < circles.size(); idx++) {
      pose2_transforms_[idx].setCost(distance_function_);
      pose2_transforms_[idx].setIndices(indices_);
      scalar_additions_[idx].setCost(&pose2_transforms_[idx]);
      smooth_minimum_.addCost(&scalar_additions_[idx]);
    }
  }
  // Reset the position of each center and update their radius.
  for (size_t idx = 0; idx < circles.size(); idx++) {
    pose2_transforms_[idx].setPose(Pose2d::Translation(circles[idx].center));
    scalar_additions_[idx].setConstant(-circles[idx].radius);
  }
}

}  // namespace planner_cost
}  // namespace isaac
