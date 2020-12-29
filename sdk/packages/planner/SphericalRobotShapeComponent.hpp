/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/types.hpp"
#include "packages/navigation/gems/robot_shape.hpp"
#include "packages/navigation/gems/spherical_robot_shape.hpp"
#include "packages/planner/RobotShapeComponent.hpp"

namespace isaac {
namespace planner {

// Model of a robot composed of a union of circles. The distance function is approximated by the
// function -ln(sum(exp(-alpha * dist_i))/alpha
// where alpha = ln(1 + #circles) * smooth_minimum controls how well the min function is
// approximated. If the real distance is D, then we have:
// D - 1/smooth_minimum <= distance <= D;
class SphericalRobotShapeComponent : public RobotShapeComponent {
 public:
  void showRobot(sight::Sop& sop) const override;

  // Return a pointer toward an object which implements a RobotShape.
  const navigation::RobotShape* robot_shape() override;

  // List of circles that compose the robot
  ISAAC_PARAM(std::vector<geometry::CircleD>, circles, {});
  // Parameters to control how well the minimum function is approximated. The error will be in the
  // range: D-1/smooth_minimum <= distance <= D where D = the real distance
  ISAAC_PARAM(double, smooth_minimum, 20.0);

 private:
  navigation::SphericalRobotShape robot_shape_;
};

}  // namespace planner
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::planner::SphericalRobotShapeComponent);
