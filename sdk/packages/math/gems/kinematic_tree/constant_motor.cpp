/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "constant_motor.hpp"

#include "packages/math/gems/kinematic_tree/motor.hpp"

namespace isaac {
namespace kinematic_tree {

ConstantMotor::ConstantMotor(const Pose3d& transformation)
    : transformation_(DualQuaternionD::FromPose3(transformation)) {}

DualQuaternionD ConstantMotor::transformation(EigenVectorConstView<double> /*state*/) const {
  return transformation_;
}

Matrix8Xd ConstantMotor::jacobian(EigenVectorConstView<double> /* state */) const {
  return Matrix8Xd(8, getNumberOfArguments());
}

int ConstantMotor::getNumberOfArguments() const {
  return 0;
}

bool ConstantMotor::isValidState(EigenVectorConstView<double> /*state*/) const {
  return true;
}

}  // namespace kinematic_tree
}  // namespace isaac
