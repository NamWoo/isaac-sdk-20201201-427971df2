/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/pose3.hpp"

namespace isaac {
namespace alice {

// A codelet which creates a 3D transformation in the pose tree between two reference frames. This
// can for example be used to set transformations which never change or to set initial values for
// transformations.
class PoseInitializer : public Codelet {
 public:
  void start() override;
  void tick() override;

  // Name of the reference frame of the left side of the pose
  ISAAC_PARAM(std::string, lhs_frame);
  // Name of the reference frame of the right side of the pose
  ISAAC_PARAM(std::string, rhs_frame);
  // Transformation lhs_T_rhs
  ISAAC_PARAM(Pose3d, pose);
  // If true reports success after initializing pose in the start function. This will make the
  // attach_interactive_marker setting invalid because the codelet won't tick.
  ISAAC_PARAM(bool, report_success, false);
  // If true, we ignore the failure when updating the PoseTree. If it's false and the update fails,
  // this codelet will report failure.
  ISAAC_PARAM(bool, disable_failure, false);
  // If enabled the pose is editable via an interactive marker.
  ISAAC_PARAM(bool, attach_interactive_marker, false);
  // Additional yaw angle around the Z axis in degrees. Currently only enabled if
  // `attach_interactive_marker` is false.
  ISAAC_PARAM(double, add_yaw_degrees, 0.0);
  // Additional pitch angle around the Y axis in degrees. Currently only enabled if
  // `attch_interactive_marker` is false.
  ISAAC_PARAM(double, add_pitch_degrees, 0.0);
  // Additional roll angle around the X axis in degrees. Currently only enabled if
  // `attch_interactive_marker` is false.
  ISAAC_PARAM(double, add_roll_degrees, 0.0);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::PoseInitializer);
