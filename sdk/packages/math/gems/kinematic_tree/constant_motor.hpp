/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/dual_quaternion.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "packages/math/gems/kinematic_tree/motor.hpp"

namespace isaac {
namespace kinematic_tree {

// Interface fpr a motor of a manipulator.
class ConstantMotor : public Motor {
 public:
  ConstantMotor(const Pose3d& transformation);

  virtual ~ConstantMotor() = default;

  DualQuaternionD transformation(EigenVectorConstView<double> /*state*/) const override;

  Matrix8Xd jacobian(EigenVectorConstView<double> /*state*/) const override;

  int getNumberOfArguments() const override;

  bool isValidState(EigenVectorConstView<double> /*state*/) const override;

 public:
  DualQuaternionD transformation_;
};

}  // namespace kinematic_tree
}  // namespace isaac
