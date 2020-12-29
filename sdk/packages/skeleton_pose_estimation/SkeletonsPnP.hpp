/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "messages/camera.capnp.h"
#include "messages/detections.capnp.h"

namespace isaac {
namespace skeleton_pose_estimation {

// This codelet reads in list of list of skeleton detections with respect to the sensor frame
// refines the pose of skeleton detection using the Perspective-n-Point (PnP) solver on
//  * 2D projections of skeleton detections corners
//  * 3D prior poses of skeleton detections corners from CAD or measurements
//
// Note: The pose tree should contain rigid transforms for skeletons_of_interest_T_skeleton_joint.
class SkeletonsPnP : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Intrinsics of color camera containing the skeleton detections
  ISAAC_PROTO_RX(CameraIntrinsicsProto, intrinsics);
  // Input list of skeleton detections with respect to the sensor frame.
  ISAAC_PROTO_RX(Skeleton2ListProto, skeletons);

  // Output list of PnP constrained reprojections skeleton with respect to the sensor frame.
  ISAAC_PROTO_TX(Skeleton2ListProto, reprojected_skeletons)
  // Output list of objects with respect to the sensor frame, their class names and confidence.
  ISAAC_PROTO_TX(Detections3Proto, detections)

  // List of comma separated skeletons of interest (i.e. "dolly").
  // The pose tree should contain transforms for skeletons_of_interest_T_skeleton_joint.
  ISAAC_PARAM(std::string, skeletons_of_interest);
  // Reprojection error threshold, pixels.
  // Note:
  //  * Skeletons with the reprojection error above set threshold will be filtered out.
  //  * Confidence is set to:
  //     (reprojection_error_threshold - reprojection_error) / reprojection_error_threshold
  ISAAC_PARAM(double, reprojection_error_threshold, 10.0);
  // Minimum number of joints threshold.
  // NOTE: pose estimates for objects associated with fewer joints will be filtered out.
  ISAAC_PARAM(int, minimum_joints_threshold, 6);
};

}  // namespace skeleton_pose_estimation
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::skeleton_pose_estimation::SkeletonsPnP);
