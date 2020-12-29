/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "prismatic_motor.hpp"

#include <cmath>
#include <limits>

#include "engine/core/assert.hpp"
#include "packages/math/gems/kinematic_tree/motor.hpp"

namespace isaac {
namespace kinematic_tree {

PrismaticMotor::PrismaticMotor(const Vector3d& axis) : axis_(axis), limits_(std::nullopt) {}

PrismaticMotor::PrismaticMotor(const Vector3d& axis, const Vector2d& limits)
    : axis_(axis), limits_(limits) {
}

DualQuaternionD PrismaticMotor::transformation(EigenVectorConstView<double> state) const {
  ASSERT(state.size() == getNumberOfArguments(), "Invalid parameters: size = %d", state.size());
  return DualQuaternionD::FromPose3(Pose3d::Translation(axis_ * state[0]));
}

Matrix8Xd PrismaticMotor::jacobian(EigenVectorConstView<double> state) const {
  ASSERT(state.size() == getNumberOfArguments(), "Invalid parameters: size = %d", state.size());
  Matrix8Xd jac = Matrix8d::Zero(8, getNumberOfArguments());
  jac.block<3, 1>(5, 0) = 0.5 * axis_;
  return jac;
}

int PrismaticMotor::getNumberOfArguments() const {
  return 1;
}

bool PrismaticMotor::isValidState(EigenVectorConstView<double> state) const {
  ASSERT(state.size() == getNumberOfArguments(), "Invalid parameters: size = %d", state.size());
  if (limits_ == std::nullopt) return true;
  const double length = state[0];
  return (*limits_)[0] <= length && length <= (*limits_)[1];
}

VectorXd PrismaticMotor::getLowerRange() const {
  return VectorXd::Constant(1, limits_ ? (*limits_)[0] : std::numeric_limits<double>::lowest());
}

VectorXd PrismaticMotor::getUpperRange() const {
  return VectorXd::Constant(1, limits_ ? (*limits_)[1] : std::numeric_limits<double>::max());
}

}  // namespace kinematic_tree
}  // namespace isaac
