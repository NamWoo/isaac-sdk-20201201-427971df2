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
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/camera.capnp.h"

namespace isaac {
namespace message_generators {

// This codelet reads parameterized camera intrinsics information and encodes CameraIntrinsicsProto
// message containing camera intrinsic information. This codelet along with ImageLoader can be
// used for example to create mock up tests when no camera hardware is available.
class CameraIntrinsicsGenerator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output message will be published with the acqtime of this input message
  ISAAC_PROTO_RX(ImageProto, image);
  // Intrinsics including pinhole and distortion parameters
  ISAAC_PROTO_TX(CameraIntrinsicsProto, intrinsics);

  // Image undistortion model. Must be 'brown' or 'fisheye'
  ISAAC_PARAM(std::string, distortion_model, "brown");
  // Focal length in pixels
  ISAAC_PARAM(Vector2d, focal_length);
  // Optical center in pixels
  ISAAC_PARAM(Vector2d, optical_center);
  // Distortion coefficients
  ISAAC_PARAM(Vector5d, distortion_coefficients, Vector5d::Zero());
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::CameraIntrinsicsGenerator);
