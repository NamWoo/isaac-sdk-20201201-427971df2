/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/sight/sop.hpp"
#include "packages/map/gems/obstacle_with_pose2.hpp"
#include "packages/map/ObstacleAtlas.hpp"
#include "packages/navigation/gems/robot_shape.hpp"

namespace isaac {
namespace planner {

// Interface to implement a robot model for a base moving in a 2 plan (3 degrees of freedom, X, Y,
// and heading).
class RobotShapeComponent : public alice::Component {
 public:
  virtual ~RobotShapeComponent() = default;

  // Render the robot in sight
  virtual void showRobot(sight::Sop& sop) const = 0;

  // Return a pointer toward an object which implements a RobotShape.
  virtual const navigation::RobotShape* robot_shape() = 0;

  // Frame of the robot model
  ISAAC_PARAM(std::string, robot_frame, "robot");
};

}  // namespace planner
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::planner::RobotShapeComponent);
