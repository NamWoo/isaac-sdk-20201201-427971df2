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

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/pose2.hpp"
#include "packages/navigation/gems/robot_shape.hpp"

namespace isaac {
namespace map {

// Base class for a flatmap (SE(2)) cost function. This can be used to determine the cost for a
// robot to be at a given location (in 2D).
class FlatmapCost : public alice::Codelet {
 public:
  virtual ~FlatmapCost() = default;

  // Returns the cost for a robot to be at a given position in the world. If the position is invalid
  // std::nullopt will be returned
  virtual double cost(const navigation::RobotShape* robot, const Pose2d& world_T_robot) const = 0;

  // Returns the penalty associated to backward motion. If invalidWeight is returned then the
  // backward motion is disabled.
  virtual double backwardPenalty(const navigation::RobotShape* robot,
                                 const Pose2d& world_T_robot) const = 0;

  // Returns a weight that represent an invalid position.
  static constexpr double invalidWeight() {
    return std::numeric_limits<double>::infinity();
  }
};

}  // namespace map
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT_BASE(isaac::map::FlatmapCost);
