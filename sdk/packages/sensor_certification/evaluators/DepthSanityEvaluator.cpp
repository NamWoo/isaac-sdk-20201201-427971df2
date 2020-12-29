/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/sensor_certification/evaluators/DepthSanityEvaluator.hpp"

#include <string>

#include "messages/camera.hpp"

namespace isaac {
namespace evaluators {

namespace {
constexpr float kTimeoutSeconds = 4.0;
};

void DepthSanityEvaluator::start() {
  tickPeriodically();
  image_message_count_ = 0;
  intrinsics_message_count_ = 0;
  expected_frames_ = get_num_frames();
  synchronize(rx_depth_listener(), rx_depth_intrinsics_listener());
}

void DepthSanityEvaluator::tick() {
  rx_depth_listener().processAllNewMessages([this] (auto proto, int64_t pubtime, int64_t acqtime) {
    if (image_message_count_ < expected_frames_) {
      image_message_count_++;
      checkMessage(proto);
    }
  });

  rx_depth_intrinsics_listener().processAllNewMessages([this] (
    auto proto, int64_t pubtime, int64_t acqtime) {
    if (intrinsics_message_count_ < expected_frames_) {
      intrinsics_message_count_++;
      checkMessage(proto);
    }
  });

  if (image_message_count_ >= expected_frames_ && intrinsics_message_count_ >= expected_frames_) {
    reportMessage("%s\n", "All images and intrinsics matched expected dimensions");
    reportSuccess();
  } else {
    checkTimeout();
  }
}

float DepthSanityEvaluator::getTimeoutSeconds() {
  return kTimeoutSeconds;
}

void DepthSanityEvaluator::reportTimeoutMessage() {
  reportMessage(
    "Timed out after %u seconds. Only received %u/%u expected image frames"
    "and %u/%u expected intrinsics messages",
    (unsigned int)kTimeoutSeconds,
    image_message_count_,
    expected_frames_,
    intrinsics_message_count_,
    expected_frames_);
}

void DepthSanityEvaluator::checkMessage(const ImageProto::Reader& image_proto) {
  int rows = image_proto.getRows();
  int cols = image_proto.getCols();

  if (rows != get_rows()) {
    reportMessage(
      "Image %u/%u height mismatch. Expected width of %u pixels but got %u.",
      image_message_count_, expected_frames_, get_rows(), rows);
    reportFailure();
    return;
  }

  if (cols != get_cols()) {
    reportMessage(
      "Image %u/%u width mismatch. Expected width of %u pixels but got %u.",
      image_message_count_, expected_frames_, get_cols(), cols);
    reportFailure();
    return;
  }
}

void DepthSanityEvaluator::checkMessage(const CameraIntrinsicsProto::Reader& intrinsics_proto) {
  float focal_x = intrinsics_proto.getPinhole().getFocal().getX();
  float focal_y = intrinsics_proto.getPinhole().getFocal().getY();

  if ((focal_x < get_focal_x()*0.99) || (focal_x > get_focal_x()*1.01)) {
    reportMessage(
      "Image %u/%u x depth focal length mismatch. Expected \"%.02f\" but got \"%.02f\".",
      intrinsics_message_count_, expected_frames_, get_focal_x(), focal_x);
    reportFailure();
    return;
  }

  if ((focal_y < get_focal_y()*0.99) || (focal_y > get_focal_y()*1.01)) {
    reportMessage(
      "Image %u/%u y depth focal length mismatch. Expected \"%.02f\" but got \"%.02f\".",
      intrinsics_message_count_, expected_frames_, get_focal_y(), focal_y);
    reportFailure();
    return;
  }
}

}  // namespace evaluators
}  // namespace isaac
