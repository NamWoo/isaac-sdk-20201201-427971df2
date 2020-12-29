/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <limits>

#include "engine/core/math/dual_quaternion.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace kinematic_tree {

// Interface for a motor of a kinematic tree.
class Motor {
 public:
  virtual ~Motor() = default;

  // Returns the dual quaternion transformation associated to the current state
  virtual DualQuaternionD transformation(EigenVectorConstView<double> state) const = 0;

  // Returns the Jacobian (8 x numberOfArguments) of the transform function above
  // The order should match the dual quaternion: (rw, rx, ry, rz,drw, dx, dy, dz).
  virtual Matrix8Xd jacobian(EigenVectorConstView<double> state) const = 0;

  // Returns how many argument the function above takes.
  virtual int getNumberOfArguments() const = 0;

  // Returns whether the state is valid for the given motor.
  virtual bool isValidState(EigenVectorConstView<double> state) const = 0;

  // Returns the lower range of each argument associated to this Motor.
  virtual VectorXd getLowerRange() const {
    return VectorXd::Constant(getNumberOfArguments(), std::numeric_limits<double>::lowest());
  }
  // Returns the upper range of each argument associated to this Motor.
  virtual VectorXd getUpperRange() const {
    return VectorXd::Constant(getNumberOfArguments(), std::numeric_limits<double>::max());
  }
};

}  // namespace kinematic_tree
}  // namespace isaac
