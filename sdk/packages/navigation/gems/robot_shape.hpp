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
#include <utility>
#include <vector>

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/types.hpp"
#include "packages/map/gems/obstacle_with_pose2.hpp"

namespace isaac {
namespace navigation {

// Interface to implement a robot model for a base moving in a 2 plane (3 degrees of freedom, X, Y,
// and heading).
class RobotShape {
 public:
  virtual ~RobotShape() = default;
  // Returns whether the robot shape at the given pose is colliding with one of the obstacles.
  // Obstacles is a list of pairs composed of the obstacle and the transformation:
  // obstacle_frame_T_world, where world is the base frame used to call these functions.
  virtual bool isColliding(const std::vector<map::ObstacleWithPose2>& obstacles,
                           const Pose2d& world_T_robot) const = 0;

  // Returns the signed distance between the robot at a given pose and the closest obstacle.
  // If the robot is colliding with an obstacle, the distance will be negative. In addition
  // if gradient and/or hessian is provided, the gradient/hessian will be populated. The first two
  // dimensions corresponds to X and Y while the third dimension correspond to the heading.
  virtual double distance(const std::vector<map::ObstacleWithPose2>& obstacles,
                          const Pose2d& world_T_robot, Vector3d* gradient = nullptr,
                          Matrix3d* hessian = nullptr) const = 0;

  // Returns whether the path composed from a direct translation of 'distance' (can be negative) in
  // the orientation of the robot is valid or not (a path is valid if it never gets closer than
  // min_distance, if min_distance is negative a path is valid if it does not collide)
  virtual bool validTranslation(const std::vector<map::ObstacleWithPose2>& obstacles,
                                const Pose2d& world_T_robot, double distance,
                                double min_distance) const = 0;

  // Returns whether the robot can rotate of 'delta_angle' (can be negative) from its current
  // position (a rotation is valid if it never gets closer than  min_distance, if min_distance is
  // negative a path is valid if it does not collide)
  virtual bool validRotation(const std::vector<map::ObstacleWithPose2>& obstacles,
                             const Pose2d& world_T_robot, double delta_angle,
                             double min_distance) const = 0;

  // Returns whether a point (in the robot coordinate frame) is inside the robot.
  virtual bool isInside(const Vector2d& point) const = 0;
};

}  // namespace navigation
}  // namespace isaac
