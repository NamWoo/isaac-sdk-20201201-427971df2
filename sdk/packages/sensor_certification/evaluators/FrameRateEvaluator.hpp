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
#include "packages/sensor_certification/evaluators/Evaluator.hpp"

namespace isaac {
namespace evaluators {

// Captures several depth or color images and verifies that the framerate is within the given
// tolerances
class FrameRateEvaluator : public Evaluator {
 public:
  void start() override;
  void tick() override;

  // Color channel to listen to for framerate
  ISAAC_PROTO_RX(ColorCameraProto, color_listener);

  // Depth channel to listen to for framerate
  ISAAC_PROTO_RX(DepthCameraProto, depth_listener);

  // Target framerate
  ISAAC_PARAM(double, target_fps, 30.0);

  // Tolerance for framerate (% error)
  ISAAC_PARAM(double, fps_tolerance, 1.0);

  // Whether to test depth frame rate, otherwise, test the color framerate
  ISAAC_PARAM(bool, test_depth, false);

  // Length of time (seconds) to run framerate test
  ISAAC_PARAM(double, test_duration, 5.0);

 protected:
  float getTimeoutSeconds() override;
  void reportTimeoutMessage() override;

 private:
  // Calculates the framerate from the stored message timestamps and verifies it is within
  // the given tolerance. Reports failure if the framerate exceeds the given tolerance.
  void calculateFrameRate();
  // The total number of frames to capture
  unsigned int expected_frames_;
  // The number of frames captured so far
  unsigned int message_count_;
  // The acqtime for all received frames
  std::vector<int64_t> message_times_;
  // Seconds test takes to timeout
  float timeout_seconds_;
};

}  // namespace evaluators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::evaluators::FrameRateEvaluator);
