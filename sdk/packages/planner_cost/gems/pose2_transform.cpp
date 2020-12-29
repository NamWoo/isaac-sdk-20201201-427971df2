/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/pose2_transform.hpp"

#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

bool Pose2Transform::isValid(double time, const VectorXd& state) {
  if (!cost_) return true;
  return cost_->isValid(time, state_);
}

double Pose2Transform::evaluate(double time, const VectorXd& state) {
  if (!cost_) return 0.0;
  populateState(state);
  return cost_->evaluate(time, state_);
}

void Pose2Transform::addGradient(double time, const VectorXd& state,
                                 Eigen::Ref<VectorXd> gradient) {
  if (!cost_) return;
  gradient_.setZero();
  populateState(state);
  cost_->addGradient(time, state_, gradient_);

  Matrix2d rotation_derivative;
  rotation_derivative << -state_as_pose_.rotation.sin(), -state_as_pose_.rotation.cos(),
                          state_as_pose_.rotation.cos(), -state_as_pose_.rotation.sin();

  Matrix3d jacobian = Matrix3d::Identity();
  jacobian.block<2, 1>(0, 2) = rotation_derivative * transform_.translation;
  gradient_ = jacobian.transpose() * gradient_;
  for (int i = 0; i < 3; i++) {
    gradient[indices_[i]] += gradient_[i];
  }
}

void Pose2Transform::addHessian(double time, const VectorXd& state,
                                Eigen::Ref<MatrixXd> hessian) {
  if (!cost_) return;
  hessian_.setZero();
  populateState(state);
  cost_->addHessian(time, state_, hessian_);

  Matrix2d rotation_derivative;
  rotation_derivative << -state_as_pose_.rotation.sin(), -state_as_pose_.rotation.cos(),
                          state_as_pose_.rotation.cos(), -state_as_pose_.rotation.sin();

  Matrix3d jacobian = Matrix3d::Identity();
  jacobian.block<2, 1>(0, 2) = rotation_derivative * transform_.translation;
  hessian_ = jacobian.transpose() * hessian_ * jacobian;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      hessian(indices_[i], indices_[j]) += hessian_(i, j);
    }
  }
}

void Pose2Transform::populateState(const VectorXd& state) {
  state_as_pose_ = Pose2d::FromXYA(state[indices_[0]], state[indices_[1]], state[indices_[2]]);
  const Pose2d pose = state_as_pose_ * transform_;
  state_.head<2>() = pose.translation;
  state_[2] = pose.rotation.angle();
}

}  // namespace planner_cost
}  // namespace isaac
