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
#include "engine/gems/serialization/json.hpp"
#include "messages/camera.capnp.h"
#include "messages/fiducial_list.capnp.h"
#include "packages/sensor_certification/evaluators/Evaluator.hpp"

namespace isaac {
namespace evaluators {

// AprilTagsEvaluator is a codelet used to assess the accuracy and precision of the fiducial
// detections made by the AprilTagsDetection codelet. It is meant to be used in conjunction with the
// driver codelet of a camera sensor, which should publish ImageProto structs, to sanity test
// whether that camera provides images that are compatible with the computer vision based algorithms
// in ISAAC SDK. As such, this codelet is only meant to be ran as part of the sensor certification
// framework which will launch it alongside the driver of the sensor under test and the
// AprilTagsDetection codelet. The verification process consists of first ensuring that fiducial
// detections are able to be made from the ImageProto structs published by the camera driver, after
// which a sequence of fiducial detections are checked to be within a pre-specified ROI which is
// currently set to be a centered area 3/4 of the height and width of the image resolution.
class AprilTagsEvaluator : public Evaluator {
 public:
  void start() override;
  void tick() override;

  // The camera image which should be of type Image3ub and in color. This is used only to save a
  // sample PNG image to the sensor certification test logs when running this test.
  ISAAC_PROTO_RX(ImageProto, color);
  // The list of april tag fiducials the test will verify are valid, meaning the fiducials have at
  // least 4 points (corners), and that based on those points the fiducials lie within an ROI.
  ISAAC_PROTO_RX(FiducialListProto, fiducials);

  // The ratio between the dimensions of the ROI to the resolution of the image provided by the
  // camera under test.
  ISAAC_PARAM(double, roi_ratio, 0.75);
  // The amount of time (seconds) the test is set to be ran for.
  ISAAC_PARAM(int, test_timeout);
  // The number of FiducialListProto messages to be captured.
  ISAAC_PARAM(int, num_frames);

 protected:
  float getTimeoutSeconds() override;
  void reportTimeoutMessage() override;

 private:
  // Setup ROI as a centered box 0.75 the resolution of image.
  void setupRoi(const ImageConstView3ub& image, double roi_ratio);
  // Extracts aligned box in image frame around fiducial detection.
  Eigen::AlignedBox2i getFiducialBox(FiducialProto::Reader fiducial);
  // Verify april tags fiducials are all fully within the ROI.
  bool fiducialsInRoi(capnp::List<FiducialProto, capnp::Kind::STRUCT>::Reader fiducial_list,
                      std::string& error_message);

  // The number of FiducialListProto messages received so far.
  unsigned int message_count_;
  // The region of interest in which fidicual detections are expected to be within.
  Eigen::AlignedBox2i roi_;
};

}  // namespace evaluators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::evaluators::AprilTagsEvaluator);

