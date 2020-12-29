/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "AprilTagsEvaluator.hpp"

#include <string>

#include "engine/core/assert.hpp"
#include "engine/gems/image/color.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"
#include "messages/camera.hpp"
#include "messages/image.hpp"
#include "messages/math.hpp"

namespace isaac {
namespace evaluators {

void AprilTagsEvaluator::start() {
  // Initialize the number of FiducialListProto messages received to zero to start counting.
  message_count_ = 0;
  tickPeriodically();
}

void AprilTagsEvaluator::tick() {
  // Read in the color image.
  ImageConstView3ub image;
  if (rx_color().available()) {
    FromProto(rx_color().getProto(), rx_color().buffers(), image);
  }

  if (!get_setup_done()) {
    setupRoi(image, get_roi_ratio());
  } else {
    // Read in the april tags fiducials list.
    if (rx_fiducials().available()) {
      auto fiducials = rx_fiducials().getProto().getFiducialList();

      // Check april tag fiducials are valid within ROI.
      message_count_++;
      std::string error_message;

      if (fiducialsInRoi(fiducials, error_message)) {
        if (message_count_ >= (unsigned int) get_num_frames()) {
          reportMessage("%s\n", "April tag detections were all within the target region in the "
                        "FOV of the camera for all frames");
          isaac::SavePng(image, get_report_directory() + "/color.png");
          reportSuccess();
        }
      } else {
        reportMessage(error_message.c_str(), message_count_, (unsigned int)get_num_frames());
        reportFailure();
      }
    }

    checkTimeout();
  }
}

// TODO change Evaluator class to define timeout as class property then remove this function
float AprilTagsEvaluator::getTimeoutSeconds() {
  return static_cast<float>(get_test_timeout());
}

void AprilTagsEvaluator::reportTimeoutMessage() {
  reportMessage(
    "Timed out after %u seconds. Only received %u/%u expected frames\n",
    (unsigned int)get_test_timeout(), message_count_, (unsigned int)get_num_frames());
}

void AprilTagsEvaluator::setupRoi(const ImageConstView3ub& image, double roi_ratio) {
  Vector2i roi_size = (image.dimensions().cast<double>() * roi_ratio).cast<int>();
  const int center_row = image.rows() / 2;
  const int center_col = image.cols() / 2;
  const int roi_top_row = center_row - (roi_size[0] / 2);
  const int roi_bottom_row = center_row + (roi_size[0] / 2);
  const int roi_left_col = center_col - (roi_size[1] / 2);
  const int roi_right_col = center_col + (roi_size[1] / 2);

  show("Region", [&](sight::Sop& sop) {
    sop.style = sight::SopStyle{"rgb(0, 255, 0)"};
    sop.add(geometry::LineSegment2d({roi_top_row, roi_left_col}, {roi_bottom_row, roi_left_col}));
    sop.add(geometry::LineSegment2d({roi_top_row, roi_left_col}, {roi_top_row, roi_right_col}));
    sop.add(geometry::LineSegment2d({roi_bottom_row, roi_right_col},
                                    {roi_top_row, roi_right_col}));
    sop.add(geometry::LineSegment2d({roi_bottom_row, roi_right_col},
                                    {roi_bottom_row, roi_left_col}));
  });

  const Vector2i roi_min = {roi_top_row, roi_left_col};
  const Vector2i roi_max = {roi_bottom_row, roi_right_col};

  roi_ = Eigen::AlignedBox2i(roi_min, roi_max);
}

Eigen::AlignedBox2i AprilTagsEvaluator::getFiducialBox(FiducialProto::Reader fiducial) {
  const auto keypoints = fiducial.getKeypoints();

  // If a detection invalid (does not have at least 4 corners) then return null box.
  if (keypoints.size() < 4) {
    return Eigen::AlignedBox2i();
  }

  Vector2d min = FromProto(keypoints[0]);
  Vector2d max = min;

  for (size_t j = 0; j < keypoints.size(); j++) {
    const auto key_point = FromProto(keypoints[j]);
    min = min.cwiseMin(key_point);
    max = max.cwiseMax(key_point);
  }

  return Eigen::AlignedBox2i(min.cast<int>(), max.cast<int>());
}

bool AprilTagsEvaluator::fiducialsInRoi(
    capnp::List<FiducialProto, capnp::Kind::STRUCT>::Reader fiducial_list,
    std::string& error_message) {
  if (fiducial_list.size() == 0) {
    error_message = "Frame %u/%u: No april tags detected in target region of FOV of camera.\n";
    reportFailure();
    return false;
  } else {
    for (size_t i = 0; i < fiducial_list.size(); i++) {
      // If fiducial is null meaning it is invalid then fail the test.
      const Eigen::AlignedBox2i fiducial_box = getFiducialBox(fiducial_list[i]);
      if (fiducial_box.isNull()) {
        error_message = "Frame %u/%u: April tag detected but detection has less than 4 corners.\n";
        reportFailure();
        return false;
      }

      if (!roi_.contains(fiducial_box)) {
        error_message = "Frame %u/%u: April tags detected but detections were not all fully "
                        "within the target region.\n";
        reportFailure();
        return false;
      }
    }
  }

  return true;
}

}  // namespace evaluators
}  // namespace isaac
