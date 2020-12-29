/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/epsilon.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace math {

// Compute the weighted average for a stream of Pose2
template <typename K>
class Pose2Average {
 public:
  // Adds another pose to the average
  void add(const Pose2<K>& pose, K weight) {
    average_translation_ += weight * pose.translation;
    average_rotation_ += weight * pose.rotation.asDirection();
    total_weight_ += weight;
  }
  void add(const Pose2<K>& pose) {
    add(pose, K(1));
  }

  // Gets the current average. Will return nullopt in case the average could not be determined
  // uniquely either because it is ambiguous or all weights so far were 0.
  std::optional<Pose2<K>> computeAverage() const {
    // Check that we got any elements with non-zero weights
    if (IsAlmostZero(total_weight_)) {
      return std::nullopt;
    }
    // Check that rotations are not antipolar. We choose a larger epsilon to compensate for
    // inaccuracies with sin/cos computations. In case rotations are antipolar every rotation
    // satisfies the criteria for a mean rotation and identiy is chosen.
    constexpr K kZeroTolerance = K(10.0) * MachineEpsilon<K>;
    const K average_rotation_norm = average_rotation_.norm();
    if (average_rotation_norm <= kZeroTolerance) {
      return Pose2<K>::Translation(average_translation_ / total_weight_);
    } else {
      return Pose2<K>{SO2<K>::FromNormalized(average_rotation_ / average_rotation_norm),
                      average_translation_ / total_weight_};
    }
  }

  // Returns the sum of all weights so far
  K total_weight() const {
    return total_weight_;
  }

 private:
  Vector2<K> average_translation_ = Vector2<K>::Zero();
  Vector2<K> average_rotation_ = Vector2<K>::Zero();
  K total_weight_ = K(0);
};

using Pose2AverageD = Pose2Average<double>;
using Pose2AverageF = Pose2Average<float>;

// Compute the weighted average for a stream of Pose3
// For the translation it's simple we can just compute the weighted average normally
// For a rotation we need to extract the eigen vector of the matrix made of the quaternions:
// More details are provided here: http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
template <typename K>
class Pose3Average {
 public:
  // Adds another pose to the average
  void add(const Pose3<K>& pose, K weight) {
    average_translation_ += weight * pose.translation;
    const Vector4<K> quat = pose.rotation.quaternion().coeffs();
    average_quaterion_matrix_ += weight * quat * quat.transpose();
    total_weight_ += weight;
  }
  void add(const Pose3<K>& pose) {
    add(pose, K(1));
  }

  // Gets the current average. Will return nullopt in case the average could not be determined
  // uniquely either because it is ambiguous or all weights so far were 0.
  std::optional<Pose3<K>> computeAverage() const {
    // Check that we got any elements with non-zero weights
    if (IsAlmostZero(total_weight_)) {
      return std::nullopt;
    }
    const Eigen::SelfAdjointEigenSolver<Matrix4<K>> solver(
        average_quaterion_matrix_ / total_weight_);
    return Pose3d{SO3d::FromQuaternion(Quaterniond(solver.eigenvectors().col(3))),
                  average_translation_ / total_weight_};
  }

  // Returns the sum of all weights so far
  K total_weight() const {
    return total_weight_;
  }

 private:
  Vector3<K> average_translation_ = Vector3<K>::Zero();
  Matrix4<K> average_quaterion_matrix_ = Matrix4<K>::Zero();
  K total_weight_ = K(0);
};

using Pose3AverageD = Pose3Average<double>;
using Pose3AverageF = Pose3Average<float>;

}  // namespace math
}  // namespace isaac
