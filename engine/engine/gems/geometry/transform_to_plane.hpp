/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <limits>

#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/geometry/pinhole.hpp"

namespace isaac {
namespace geometry {

// The function projects a point in the pixel space to a z-plane on the target frame.
// - pixel           is the coordinate of the object in the pixel space
// - pinhole camera  contains the intrinsic parameters of the pinhole camera
// - plane_T_camera  defines the transformation from camera's frame to the target frame
// - height          defines the height of the z-plane in the target frame
// The valid value is only returned when the object is in front of the camera
template <typename K>
std::optional<Vector3<K>> TransformToPlane(
    const Vector2<K>& pixel, const geometry::Pinhole<K>& pinhole_camera,
    const Pose3<K>& target_T_camera, const K height = K(0)) {
  // u_target: the unit vector of the ray from camera to object (vector)
  // a_target: the translation from target frame to camera frame (vector)
  // (both are expressed in the target frame)
  const Vector3<K> u_target = target_T_camera.rotation * pinhole_camera.unproject(pixel, K(1.0));
  const Vector3<K>& a_target = target_T_camera.translation;

  // eliminate "divide by zero" scenario
  if (std::fabs(u_target[2]) <= std::numeric_limits<K>::epsilon())
    return std::nullopt;

  // compute the vector length of the ray
  const K lambda = (height - a_target[2]) / u_target[2];
  if (lambda < K(0))  // not valid when the object is at the back of the camera
    return std::nullopt;

  return Vector3<K>{a_target + lambda * u_target};
}

}  // namespace geometry
}  // namespace isaac
