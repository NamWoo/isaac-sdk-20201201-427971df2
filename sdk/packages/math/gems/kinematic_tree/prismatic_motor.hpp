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
#include "engine/core/optional.hpp"
#include "packages/math/gems/kinematic_tree/motor.hpp"

namespace isaac {
namespace kinematic_tree {

// Prismtatic joint motor (a.k.a. Slide joint) that encodes a translation motion along an axis.
// This motor requires a single parameter from the state space: the length of the slide along the
// provided axis.
class PrismaticMotor : public Motor {
 public:
  // Constructs a PrismaticMotor with the axis.
  PrismaticMotor(const Vector3d& axis);
  // Constructs a PrismaticMotor with the axis and also provides the limits of the length for this
  // joint.
  // Note: if the provided axis is not normalized, the scale factor will be applied to all the
  // transformations.
  PrismaticMotor(const Vector3d& axis, const Vector2d& limits);

  virtual ~PrismaticMotor() = default;

  DualQuaternionD transformation(EigenVectorConstView<double> state) const override;

  Matrix8Xd jacobian(EigenVectorConstView<double> state) const override;

  int getNumberOfArguments() const override;

  bool isValidState(EigenVectorConstView<double> state) const override;

  VectorXd getLowerRange() const override;

  VectorXd getUpperRange() const override;

 public:
  Vector3d axis_;
  std::optional<Vector2d> limits_;
};

}  // namespace kinematic_tree
}  // namespace isaac
