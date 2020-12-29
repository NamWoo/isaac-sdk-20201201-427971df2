/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <random>

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/so2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/gems/interpolation/linear.hpp"

namespace isaac {

// Interpolates between two 2D poses
// This uses "independent" interpolation of translation and rotation. This is only one of multiple
// ways to interpolate between two rigid body poses.
template <typename K>
Pose2<K> Interpolate(K p, const Pose2<K>& a, const Pose2<K>& b) {
  return Pose2<K>{
    Interpolate(p, a.rotation, b.rotation),
    Interpolate(p, a.translation, b.translation)
  };
}

// Interpolates between two 2D rotations. Due to the nature of rotations this function is
// problematic if the two rotations are about 180 degree apart. In that case small deviations in
// the input may have large deviations in the output.
template <typename K>
SO2<K> Interpolate(K p, const SO2<K>& a, const SO2<K>& b) {
  const K a0 = a.angle();
  const K a1 = b.angle();
  return SO2<K>::FromAngle(a0 + p*WrapPi(a1 - a0));
}

// Like Interpolate for Pose2
template <typename K>
Pose3<K> Interpolate(K p, const Pose3<K>& a, const Pose3<K>& b) {
  return Pose3<K>{
    Interpolate(p, a.rotation, b.rotation),
    Interpolate(p, a.translation, b.translation)
  };
}

// Like Interpolate for SO2
template <typename K>
SO3<K> Interpolate(K p, const SO3<K>& a, const SO3<K>& b) {
  return SO3<K>::FromQuaternion(a.quaternion().slerp(p, b.quaternion()));
}

}  // namespace isaac
