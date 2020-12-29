/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/pose2.hpp"
#include "packages/map/gems/obstacle.hpp"

namespace isaac {
namespace map {

// Object containgin an obstacle and the transformation between this pose and a reference frame
struct ObstacleWithPose2 {
  // Pointer to an instance of obstacle
  const Obstacle* obstacle;
  // Transformation from a reference frame to the obstacle frame.
  Pose2d obstacle_T_reference;
};

}  // namespace map
}  // namespace isaac
