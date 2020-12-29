/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "spherical_robot_shape.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "engine/core/math/types.hpp"
#include "packages/map/gems/obstacle.hpp"

namespace isaac {
namespace navigation {

bool SphericalRobotShape::isInside(const Vector2d& point) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& circle : circles_) {
    if ((circle.center - point).squaredNorm() <= std::pow(circle.radius, 2)) {
      return true;
    }
  }
  return false;
}

bool SphericalRobotShape::validTranslation(const std::vector<map::ObstacleWithPose2>& obstacles,
                                           const Pose2d& reference_T_robot, double distance,
                                           double min_distance) const {
  min_distance = std::max(parameters_.error_margin, min_distance);
  const double sign = distance < 0.0 ? -1.0 : 1.0;
  for (double p = 0.0; p <= sign * distance;) {
    const Pose2d pose = reference_T_robot * Pose2d::Translation(sign * p, 0.0);
    const double dist_to_obstacle = this->distance(obstacles, pose);
    if (dist_to_obstacle < min_distance) {
      return false;
    }
    p += std::max(parameters_.min_translation_increment, dist_to_obstacle - min_distance);
  }

  return true;
}

bool SphericalRobotShape::validRotation(const std::vector<map::ObstacleWithPose2>& obstacles,
                                        const Pose2d& reference_T_robot, double delta_angle,
                                        double min_distance) const {
  min_distance = std::max(parameters_.error_margin, min_distance);
  const double outer_radius_inv = 1.0 / outer_radius_;
  const double sign = delta_angle < 0.0 ? -1.0 : 1.0;
  for (double angle = 0.0; angle <= sign * delta_angle;) {
    const Pose2d pose = reference_T_robot * Pose2d::Rotation(sign * angle);
    const double dist_to_obstacle = this->distance(obstacles, pose);
    if (dist_to_obstacle < min_distance) {
      return false;
    }
    // Adding distance / radius is safe as every point of the robot moves by a distance less than
    // r*theta where r is the distance to the origin.
    angle += std::max(parameters_.min_rotation_increment,
                      (dist_to_obstacle - min_distance) * outer_radius_inv);
  }

  return true;
}

bool SphericalRobotShape::isColliding(const std::vector<map::ObstacleWithPose2>& obstacles,
                                      const Pose2d& reference_T_robot) const {
  std::vector<geometry::CircleD> circles;
  { // Make a copy of circles to avoid race access to it.
    std::lock_guard<std::mutex> lock(mutex_);
    circles = circles_;
  }
  for (const auto& obstacle : obstacles) {
    const Pose2d obstacle_T_robot = obstacle.obstacle_T_reference * reference_T_robot;
    for (const auto& circle : circles) {
      if (obstacle.obstacle->distance(obstacle_T_robot * circle.center) <= circle.radius) {
        return true;
      }
    }
  }
  return false;
}

// `distance` uses a smooth `min` function approximation:
// Given a list of distances d_ij, distance between the circle _i and the obstacles _j
// D = 1/alpha  * ln(sum(exp(alpha*d_ij)))
// Gradient:
// Let's call sum_exp = sum(exp(alpha*d_ij))
// dD/dx = sum(d(d_ij)/dx * exp(alpha*d_ij)) / sum_exp
// dD/dy = sum(d(d_ij)/dy * exp(alpha*d_ij)) / sum_exp
// dD/da = sum(d(d_ij)/da * exp(alpha*d_ij)) / sum_exp
// Hessian:
// Let's call gx = sum(d(d_ij)/dx) and similarly for gy and ga
//     gyx = gxy = sum(d(d_ij)/dx * d(d_ij)/dy) (same for gxx, gyy, gaa, gxa, and gya)
//     hyx = hxy = sum(d^2(d_ij)/dxdy) (same for hxx, hyy, haa, hxa, and hya)
// d^2D/dxdy = ((alpha * gxy + hxy) / sum_exp - alpha * gx * gy)
//
// To prevent numerical instability (overflow due to std::exp), we extract the minimum distance
// and remove it from all the distance. It does not change the computation of any derivative, but
// the smooth minimum is now: min_dist + log(sum(exp(alpha*(dist - min_dist)))) / alpha.
double SphericalRobotShape::distance(
    const std::vector<map::ObstacleWithPose2>& obstacles,
    const Pose2d& reference_T_robot, Vector3d* out_gradient, Matrix3d* out_hessian) const {
  std::vector<geometry::CircleD> circles;
  { // Make a copy of circles to avoid race access to it.
    std::lock_guard<std::mutex> lock(mutex_);
    circles = circles_;
  }
  // Prepare the return values
  double sum_exp = 0.0;
  if (out_gradient) out_gradient->setZero();
  if (out_hessian) out_hessian->setZero();
  // Get the constant parameter needed
  const double alpha = -parameters_.minimum_smoothing * std::log(1.0 + static_cast<double>(
      circles.size() * obstacles.size()));

  const bool compute_hessian = !parameters_.use_zero_hessian && out_hessian != nullptr;

  Vector3d sum_gradient;
  Matrix3d sum_gradient_product;
  Matrix3d sum_hessian;

  if (out_hessian || out_gradient) {
    sum_gradient.setZero();
    sum_gradient_product.setZero();
    sum_hessian.setZero();
  }

  Matrix2d rotation_derivative;
  rotation_derivative << -reference_T_robot.rotation.sin(), -reference_T_robot.rotation.cos(),
                          reference_T_robot.rotation.cos(), -reference_T_robot.rotation.sin();

  std::vector<Vector2d> centers(circles.size());
  std::vector<double> distances;
  std::vector<Vector3d> gradients;
  std::vector<Matrix3d> hessians;
  // Reserve the memory needed.
  distances.reserve(circles.size() * obstacles.size());
  gradients.reserve(circles.size() * obstacles.size());
  if (compute_hessian) {
    hessians.reserve(circles.size() * obstacles.size());
  }

  // Move the allocation outside of the loop to prevent constant allocation.
  std::vector<double> inloop_distances;
  std::vector<Vector2d> inloop_gradients;
  std::vector<Matrix2d> inloop_hessians;
  for (const auto& obstacle : obstacles) {
    const Pose2d& obstacle_T_reference = obstacle.obstacle_T_reference;
    const Pose2d obstacle_T_robot = obstacle_T_reference * reference_T_robot;
    for (size_t idx = 0; idx < circles.size(); idx++) {
      centers[idx] = obstacle_T_robot * circles[idx].center;
    }
    obstacle.obstacle->batchDistance(centers, inloop_distances);
    for (size_t idx = 0; idx < circles.size(); idx++) {
      distances.push_back(inloop_distances[idx] - circles[idx].radius);
    }

    if (out_gradient || out_hessian) {
      // Gradient of F(T(X)) = grad_F * J_T where J_T is the jacobian of the transformation T
      // Hessian of F(T(X)) = J_T^t * H_F * J_T
      obstacle.obstacle->batchGradient(centers, inloop_gradients);
      if (compute_hessian) {
        obstacle.obstacle->batchHessian(centers, inloop_hessians);
      }
      for (size_t idx = 0; idx < circles.size(); idx++) {
        // Jacobian of obstacle_T_reference * X * circles[idx].center
        Matrix<double, 2, 3> jacobian;
        jacobian.block<2, 2>(0, 0) = obstacle_T_reference.rotation.matrix();
        jacobian.block<2, 1>(0, 2) =
            obstacle_T_reference.rotation.matrix() * rotation_derivative * circles[idx].center;

        gradients.push_back(jacobian.transpose() * inloop_gradients[idx]);
        if (compute_hessian) {
          hessians.push_back(jacobian.transpose() * inloop_hessians[idx] * jacobian);
        }
      }
    }
  }
  const double min_distance =
      distances.empty() ? 0.0 : *std::min_element(distances.begin(), distances.end());
  for (size_t idx = 0; idx < distances.size(); idx++) {
    const double weight = std::exp(alpha * (distances[idx] - min_distance));
    sum_exp += weight;

    if (out_gradient || out_hessian) {
      sum_gradient += weight * gradients[idx];
      sum_gradient_product += weight * gradients[idx] * gradients[idx].transpose();
      if (compute_hessian) {
        sum_hessian += weight * hessians[idx];
      }
    }
  }

  if (IsAlmostZero(sum_exp)) return std::numeric_limits<double>::max();
  sum_gradient /= sum_exp;
  if (out_gradient) *out_gradient = sum_gradient;
  if (out_hessian) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        (*out_hessian)(i, j) = (alpha * sum_gradient_product(i, j) + sum_hessian(i, j)) / sum_exp
                             - alpha * sum_gradient(i) * sum_gradient(j);
      }
    }
  }
  return min_distance + std::log(sum_exp) / alpha;
}

}  // namespace navigation
}  // namespace isaac
