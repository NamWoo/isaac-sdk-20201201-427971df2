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
#include "packages/sensor_certification/evaluators/Evaluator.hpp"

namespace isaac {
namespace evaluators {

// Receives color camera messages and verifies the dimensions, colorspace, and focal length match
// the expected values
class ColorSanityEvaluator : public Evaluator {
 public:
  void start() override;
  void tick() override;

  // Color images to validate
  ISAAC_PROTO_RX(ImageProto, color_listener);
  ISAAC_PROTO_RX(CameraIntrinsicsProto, color_intrinsics_listener);

  // Expected number of rows
  ISAAC_PARAM(int, rows);

  // Expected number of columns
  ISAAC_PARAM(int, cols);

  // Expected number of channels
  ISAAC_PARAM(int, channels);

  // Expected focal length
  ISAAC_PARAM(float, focal_x);
  ISAAC_PARAM(float, focal_y);

  // Number of frames to test
  ISAAC_PARAM(int, num_frames);

  // Colorspace
  ISAAC_PARAM(std::string, colorspace);

 protected:
  // Override timeout
  float getTimeoutSeconds() override;
  void reportTimeoutMessage() override;

 private:
  // Checks an image message and reports failure if it doesn't match
  void checkMessage(const ImageProto::Reader& proto);
  // Checks an intrinsics message and reports failure if it doesn't match
  void checkMessage(const CameraIntrinsicsProto::Reader& proto);
  // The number of image messages received so far
  unsigned int image_message_count_;
  // The number of intrinsics messages received so far
  unsigned int intrinsics_message_count_;
  // The total number of messages to capture
  unsigned int expected_frames_;
};

}  // namespace evaluators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::evaluators::ColorSanityEvaluator);
