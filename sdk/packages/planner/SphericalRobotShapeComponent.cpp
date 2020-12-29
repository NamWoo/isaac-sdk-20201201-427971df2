/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SphericalRobotShapeComponent.hpp"

#include "engine/core/math/types.hpp"
#include "packages/navigation/gems/robot_shape.hpp"

namespace isaac {
namespace planner {

const navigation::RobotShape* SphericalRobotShapeComponent::robot_shape() {
  robot_shape_.set_minimum_smoothing(get_smooth_minimum());
  robot_shape_.set_circles(get_circles());
  return &robot_shape_;
}

void SphericalRobotShapeComponent::showRobot(sight::Sop& sop) const {
  sop.style = sight::SopStyle{"#00f", true};
  sop.transform = sight::SopTransform{this->get_robot_frame()};
  const auto circles = get_circles();
  for (const auto& circle : circles) {
    sop.add(circle);
  }
}

}  // namespace planner
}  // namespace isaac
