/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <mutex>
#include <utility>
#include <vector>

#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/types.hpp"
#include "packages/navigation/gems/robot_shape.hpp"

namespace isaac {
namespace navigation {

// Model of a robot composed of a union of circles. The distance function is approximated by the
// function -ln(sum(exp(-alpha * dist_ij))/alpha
// where dist_ij = distance(circle_i, obstacle_j) and alpha = ln(1 + #circles) * minimum_smoothing
// controls how well the min function is approximated. If the real distance is D, then we have:
// D - 1/minimum_smoothing <= distance <= D;
class SphericalRobotShape : public RobotShape {
 public:
  // List of parameters of the robot shape
  struct Parameter {
    // Parameters to control how well the minimum function is approximated. The error will be in the
    // range: D-1/minimum_smoothing <= distance <= D where D = the real distance.
    // Note: if the value is too big, while in theory it produces a better accuracy, there is a risk
    // that intermediate computation overflow what a double can hold. In general the accuracy is
    // much better than the worst case, therefore a value of ~10 would produce an accuracy of a
    // couple of centimeters while limiting the risk of overflow.
    double minimum_smoothing = 20.0;
    // Minimum angle increment used in validRotation
    double min_rotation_increment = DegToRad(1.0);
    // Minimum distance increment in validTranslation
    double min_translation_increment = 0.05;
    // Minimum distance the robot needs to keep to consider the position as not colliding.
    double error_margin = 0.0;
    // If set to true, the hessian of each obstacle will be assumed to be zero.
    // The distance function is unfortunately not differentiable, we are approximating it in order
    // to keep it differentiable. One down side is, the better the approximation the more ill
    // conditionned are the hessian. This can produce issues therefore this flag allow using a zero
    // hessian (which is the correct hessian of the distance to a line, and a good approximation of
    // the hessian of the distance from a point when the point is far enough).
    bool use_zero_hessian = true;
  };

  bool isColliding(const std::vector<map::ObstacleWithPose2>& obstacles,
                   const Pose2d& reference_T_robot) const override;

  double distance(const std::vector<map::ObstacleWithPose2>& obstacles,
                  const Pose2d& reference_T_robot, Vector3d* gradient = nullptr,
                  Matrix3d* hessian = nullptr) const override;

  bool validTranslation(const std::vector<map::ObstacleWithPose2>& obstacles,
                        const Pose2d& reference_T_robot, double distance,
                        double min_distance) const override;

  bool validRotation(const std::vector<map::ObstacleWithPose2>& obstacles,
                     const Pose2d& reference_T_robot, double delta_angle,
                     double min_distance) const override;


  bool isInside(const Vector2d& point) const override;

  // Updates the list of circles that composed the robot
  void set_circles(std::vector<geometry::CircleD> circles) {
    // The outer_radius_ needs to be greater than 0
    outer_radius_ = MachineEpsilon<double>;
    for (auto& circle : circles) {
      outer_radius_ = std::max(outer_radius_, circle.center.norm() + circle.radius);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    circles_ = std::move(circles);
  }

  // Updates the smoothing parameters to approximate the `min` function. Must be positive to
  // approximate the `min` function. See comment below for more details about the meaning.
  void set_minimum_smoothing(double minimum_smoothing) {
    parameters_.minimum_smoothing = minimum_smoothing;
  }

  // Gets the minimum radius of a circle centered at the origin that contains fully the robot.
  double getOuterRadius() const {
    return outer_radius_;
  }

  // Returns the list of parameters
  Parameter& parameters() {
    return parameters_;
  }

  const Parameter& parameters() const {
    return parameters_;
  }

 private:
  Parameter parameters_;
  // List of circles that compose the robot
  std::vector<geometry::CircleD> circles_;
  // Hold the minimum radius of a circle center in the origin that contains the robot.
  double outer_radius_ = 0.0;

  mutable std::mutex mutex_;
};

}  // namespace navigation
}  // namespace isaac

