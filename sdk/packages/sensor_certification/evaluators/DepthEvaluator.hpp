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

// Captures several depth frames and examines the mean and standard deviation of the depth
// within a target region. Verifies the mean and standard deviation are within the given
// tolerances
class DepthEvaluator : public Evaluator {
 public:
  void start() override;
  void tick() override;

  // Depth messages to evaluate
  ISAAC_PROTO_RX(ImageProto, depth_listener);

  // After setup is complete, the first color message received on this channel is saved as a png
  // to record the scene setup
  ISAAC_PROTO_RX(ImageProto, color_listener);

  // The frame rate of the depth channel, used to determine how many frames to evaluate
  ISAAC_PARAM(double, frame_rate);

  // Image dimensions
  ISAAC_PARAM(int, image_height);
  ISAAC_PARAM(int, image_width);

  // Target depth value in meters
  ISAAC_PARAM(double, target_depth, 0.5);

  // Width of target area in pixels
  ISAAC_PARAM(int, target_width, 10);

  // Height of target area in pixels
  ISAAC_PARAM(int, target_height, 10);

  // Tolerance for average depth (% error)
  ISAAC_PARAM(double, mean_tolerance, 1.0);

  // Tolerance for coefficient of variation (standard deviation / mean)
  ISAAC_PARAM(double, cov_tolerance, 0.05);

  // Direction to move target: 0-none, 1-up, 2-down, 3-left, 4-right, 5-done
  ISAAC_PARAM(int, target_cmd, 0);

 protected:
  float getTimeoutSeconds() override;
  void reportTimeoutMessage() override;

 private:
  // Saves a depth image as a contiguous array of binary encoded floats
  void saveDepthImage(const ImageConstView1f& image, uint32_t frame);
  // Saves a color image as a png
  void saveColorImage(const ImageConstView3ub& image);
  // Returns the location of target corners
  void getTargetDimensions(Vector<double, 2>& top_right,
                           Vector<double, 2>& bottom_right,
                           Vector<double, 2>& bottom_left);
  // Draws target in ISAAC Sight based on location of target
  void drawTarget();
  // The number of depth image capture after setup finished
  unsigned int message_count_;
  // The total number of depth images to evaluate
  unsigned int expected_frames_;
  // Whether or not a color image has been saved yet after scene setup
  bool color_image_saved_;
  // Pixel locaiton of top left corner of target
  Vector<double, 2> target_top_left;
};

}  // namespace evaluators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::evaluators::DepthEvaluator);
