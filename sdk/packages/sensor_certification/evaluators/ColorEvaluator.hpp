/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <unordered_map>

#include "engine/alice/alice_codelet.hpp"
#include "messages/camera.capnp.h"
#include "packages/sensor_certification/evaluators/Evaluator.hpp"

namespace isaac {
namespace evaluators {

/*
  Receives RGB color images from the camera and verifies
  whether the target region is the expected color
*/
class ColorEvaluator : public Evaluator {
 public:
  void start() override;
  void tick() override;

  // Color images to validate
  ISAAC_PROTO_RX(ImageProto, color_listener);

  // CV Masked image
  ISAAC_PROTO_TX(ImageProto, cv_image);

  // The frame rate of the color channel
  ISAAC_PARAM(double, frame_rate);

  // Image dimensions
  ISAAC_PARAM(int, image_height);
  ISAAC_PARAM(int, image_width);

  // Expected lego color of target area: 0-red, 1-green, 2-blue
  ISAAC_PARAM(int, target_color, 0);

  // Width of target area in pixels
  ISAAC_PARAM(int, target_width, 10);

  // Height of target area in pixels
  ISAAC_PARAM(int, target_height, 10);

  // Direction to move target: 0-none, 1-up, 2-down, 3-left, 4-right, 5-done
  ISAAC_PARAM(int, target_cmd, 0);

 protected:
  // Override timeout
  float getTimeoutSeconds() override;
  void reportTimeoutMessage() override;

 private:
  // Saves a color image as a png
  void saveColorImage(const ImageConstView3ub& image);
  // Returns the location of target corners
  void getTargetDimensions(Vector<double, 2>& top_right,
                           Vector<double, 2>& bottom_right,
                           Vector<double, 2>& bottom_left);
  // Calculates the euclidean distance between 2 points on a cartesian plane
  double calcDistance(double x1, double x2,
                     double y1, double y2,
                     double z1 = 0, double z2 = 0);
  // Draws target in ISAAC Sight based on location of target
  void drawTarget();
  // The number of image messages received so far
  unsigned int message_count_;
  // The total number of messages to capture
  unsigned int expected_frames_;
  // Whether or not a color image has been saved yet after scene setup
  bool color_image_saved_;
  // Pixel location of top left corner of target
  Vector<double, 2> target_top_left;
  // Map from color index to name
  std::unordered_map<int, const char*> colors;
  // Color index of detected color
  int max_idx;
};

}  // namespace evaluators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::evaluators::ColorEvaluator);
