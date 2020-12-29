/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/distance_quadratic_cost.hpp"

#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

namespace {
// Used to compute a smooth absolute speed.
constexpr double kEpsilon = 1e-4;
constexpr int kPosX = 0;
constexpr int kPosY = 1;
constexpr int kHeading = 2;
constexpr int kSpeed = 3;
}  // namespace

bool DistanceQuadraticCost::isValid(double time, const VectorXd& state) {
  if (distance_function_ == nullptr || costs_.empty()) return true;
  const Vector3d position(state[indices_[kPosX]], state[indices_[kPosY]],
                          state[indices_[kHeading]]);
  return distance_function_->isValid(time, position);
}

double DistanceQuadraticCost::evaluate(double time, const VectorXd& state) {
  if (distance_function_ == nullptr || costs_.empty()) return 0.0;
  const Vector3d position(state[indices_[kPosX]], state[indices_[kPosY]],
                          state[indices_[kHeading]]);
  const double distance = distance_function_->evaluate(time, position);
  const double speed = std::sqrt(state[indices_[kSpeed]]*state[indices_[kSpeed]] + kEpsilon);
  double total_cost = 0.0;
  for (const auto& cost : costs_) {
    const double target = cost.target_distance + speed * cost.speed_gradient;
    const double diff = distance - target;
    if (diff < 0.0) {
      total_cost += cost.gain * diff * diff;
    }
  }
  return 0.5 * total_cost;
}

void DistanceQuadraticCost::addGradient(double time, const VectorXd& state,
                                        Eigen::Ref<VectorXd> gradient) {
  if (distance_function_ == nullptr || costs_.empty()) return;
  const Vector3d position(state[indices_[kPosX]], state[indices_[kPosY]],
                          state[indices_[kHeading]]);
  Vector3d tmp_gradient = Vector3d::Zero();
  const double distance = distance_function_->evaluate(time, position);
  distance_function_->addGradient(time, position, tmp_gradient);
  const double speed = std::sqrt(state[indices_[kSpeed]]*state[indices_[kSpeed]] + kEpsilon);

  double sum_gain_diff = 0.0;
  for (const auto& cost : costs_) {
    const double target = cost.target_distance + speed * cost.speed_gradient;
    const double diff = distance - target;
    if (diff < 0.0) {
      sum_gain_diff += cost.gain * diff;
      if (cost.speed_gradient > 0.0) {
        const double gspeed = -cost.speed_gradient * state[indices_[kSpeed]] / speed;
        gradient[indices_[kSpeed]] += cost.gain * diff * gspeed;
      }
    }
  }
  for (int i = 0; i < tmp_gradient.size(); i++) {
    gradient[indices_[i]] += sum_gain_diff * tmp_gradient[i];
  }
}

void DistanceQuadraticCost::addHessian(double time, const VectorXd& state,
                                       Eigen::Ref<MatrixXd> hessian) {
  if (distance_function_ == nullptr || costs_.empty()) return;
  const Vector3d position(state[indices_[kPosX]], state[indices_[kPosY]],
                          state[indices_[kHeading]]);
  Vector3d tmp_gradient = Vector3d::Zero();
  Matrix3d tmp_hessian = Matrix3d::Zero();
  const double distance = distance_function_->evaluate(time, position);
  distance_function_->addGradient(time, position, tmp_gradient);
  if (!use_zero_hessian_) {
    distance_function_->addHessian(time, position, tmp_hessian);
  }
  const double speed = std::sqrt(state[indices_[kSpeed]]*state[indices_[kSpeed]] + kEpsilon);

  double sum_gain = 0.0;
  double sum_gain_diff = 0.0;
  Matrix4d final_matrix = Matrix4d::Zero();
  for (const auto& cost : costs_) {
    const double target = cost.target_distance + speed * cost.speed_gradient;
    const double diff = distance - target;
    if (diff < 0.0) {
      sum_gain += cost.gain;
      sum_gain_diff += cost.gain * diff;
      if (cost.speed_gradient > 0.0) {
        const double gspeed = -cost.speed_gradient * state[indices_[kSpeed]] / speed;
        final_matrix(kSpeed, kSpeed) +=
            cost.gain * (gspeed * gspeed -
                         diff * cost.speed_gradient * kEpsilon / (speed * speed * speed));
        final_matrix.col(kSpeed).head<3>() += cost.gain * gspeed * tmp_gradient;
        final_matrix.row(kSpeed).head<3>() += cost.gain * gspeed * tmp_gradient.transpose();
      }
    }
  }
  final_matrix.topLeftCorner<3, 3>() = sum_gain_diff * tmp_hessian +
                                      sum_gain * tmp_gradient * tmp_gradient.transpose();
  for (int i = 0; i < indices_.size(); i++) {
    for (int j = 0; j < indices_.size(); j++) {
      hessian(indices_[i], indices_[j]) += final_matrix(i, j);
    }
  }
}

}  // namespace planner_cost
}  // namespace isaac
