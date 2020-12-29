/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/sensor_certification/evaluators/DepthEvaluator.hpp"

#include <fstream>
#include <string>

#include "engine/gems/image/color.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"
#include "messages/camera.hpp"
#include "messages/image.hpp"

namespace isaac {
namespace evaluators {

namespace {

constexpr float kTestTime = 1.0;
constexpr float kTimeoutSeconds = kTestTime*20.0;
constexpr uint32_t kBufferSize = 4096;
constexpr int kMovementSpeed = 5;

// Get the mean depth for a rectangular region in an image
float getMeanDepth(const ImageConstView1f& image, int start_row,
                   int start_col, int width, int height) {
  float sum = 0;
  for (int row = start_row; row < start_row + height; row++) {
    for (int col = start_col; col < start_col + width; col++) {
      sum += image(row, col);
    }
  }
  return sum / (width*height);
}

// Get the standard deviation for the depth in a rectangular region in an image
float getStandardDeviation(const ImageConstView1f& image, float mean,
                           int start_row, int start_col, int width, int height) {
  float sum = 0;
  for (int row = start_row; row < start_row + height; row++) {
    for (int col = start_col; col < start_col + width; col++) {
      float value = image(row, col) - mean;
      sum += value*value;
    }
  }
  return std::sqrt(sum/(width*height));
}

}  // namespace

void DepthEvaluator::start() {
  message_count_ = 0;
  expected_frames_ = get_frame_rate()*kTestTime;
  color_image_saved_ = false;

  // initialize the target location at the center of the image
  int left_edge = get_image_width()/2 - get_target_width() / 2;
  int top_edge = get_image_height()/2 - get_target_height()/2;
  target_top_left = {top_edge, left_edge};

  drawTarget();
  tickPeriodically();
}

void DepthEvaluator::tick() {
  if (get_target_cmd() != 5) {
    // If user is still adjusting target and setup then clear depth messages
    rx_depth_listener().processAllNewMessages([this] (auto proto, int64_t pubtime,
                                                      int64_t acqtime) {});

    // Set target location based on move direction specified by user
    switch (get_target_cmd()) {
      case 1:
        target_top_left = {target_top_left.x()-kMovementSpeed, target_top_left.y()};
        break;
      case 2:
        target_top_left = {target_top_left.x()+kMovementSpeed, target_top_left.y()};
        break;
      case 3:
        target_top_left = {target_top_left.x(), target_top_left.y()-kMovementSpeed};
        break;
      case 4:
        target_top_left = {target_top_left.x(), target_top_left.y()+kMovementSpeed};
        break;
    }

    drawTarget();

    // Reset target move command
    set_target_cmd(0);

    return;
  }

  // Save the first color image once setup is done to the report directory
  if (!color_image_saved_) {
    rx_color_listener().processLatestNewMessage([this] (auto proto, int64_t pubtime,
                                                        int64_t acqtime) {
      ImageConstView3ub image;
      FromProto(proto, rx_color_listener().buffers(), image);
      saveColorImage(image);
      color_image_saved_ = true;
    });
  }

  // Evaluate the depth match of each depth message
  rx_depth_listener().processAllNewMessages([this] (auto proto, int64_t pubtime, int64_t acqtime) {
    message_count_++;

    ImageConstView1f image;
    FromProto(proto, rx_depth_listener().buffers(), image);
    saveDepthImage(image, message_count_);

    uint32_t top_left_col = target_top_left.y();
    uint32_t top_left_row = target_top_left.x();

    // Calculate the mean and std of the target region
    const float mean = getMeanDepth(image, top_left_row, top_left_col, get_target_width(),
                                    get_target_height());
    const float std = getStandardDeviation(image, mean, top_left_row, top_left_col,
                                           get_target_width(), get_target_height());
    const float cov = std / mean;
    reportMessage("Frame %u/%u: mean = %f, coefficient of variation = %f\n",
      message_count_,
      expected_frames_,
      mean,
      cov);

    // Evaluate the cov and see if it is within the tolerance
    const float cov_tolerance = get_cov_tolerance();
    if (cov > cov_tolerance) {
      reportMessage("Frame %u/%u: cov (%f) is greater than tolerance of %f\n",
        message_count_,
        expected_frames_,
        cov,
        cov_tolerance);
      reportFailure();
    }

    // Evaluate the mean and see if it is within tolerance
    const float mean_tolerance = get_mean_tolerance();
    const float mean_error = 100*std::abs(mean - get_target_depth()) / get_target_depth();
    if (mean_error > mean_tolerance) {
      reportMessage("Frame %u/%u: depth error (%f%%) is greater than tolerance of %f%%\n",
        message_count_,
        expected_frames_,
        mean_error,
        mean_tolerance);
      reportFailure();
    }
  });

  if (message_count_ >= expected_frames_) {
    reportMessage("%s\n", "All frames were within expected tolerances");
    reportSuccess();
    return;
  }

  checkTimeout();
}

float DepthEvaluator::getTimeoutSeconds() {
  return kTimeoutSeconds;
}

void DepthEvaluator::reportTimeoutMessage() {
  reportMessage(
    "Timed out after %u seconds. Only received %u/%u expected frames\n",
    (unsigned int)kTimeoutSeconds,
    message_count_,
    expected_frames_);
}

void DepthEvaluator::saveDepthImage(const ImageConstView1f& image, uint32_t frame) {
  std::ofstream stream(get_report_directory() + "/depth_" + std::to_string(frame) + ".raw",
                       std::ios::binary);
  for (auto it = image.element_wise_begin(); it != image.element_wise_end(); it++) {
    float value = *it;
    stream.write(reinterpret_cast<const char*>(&value), sizeof(float));
  }
}

void DepthEvaluator::saveColorImage(const ImageConstView3ub& image) {
  std::string name = get_report_directory() + "/color.png";
  isaac::SavePng(image, name);
}

// Calculates remaining corners of target based on top left class variable
void DepthEvaluator::getTargetDimensions(Vector<double, 2>& top_right,
                                         Vector<double, 2>& bottom_right,
                                         Vector<double, 2>& bottom_left) {
  // Calculate target edge locations
  int left_edge = target_top_left.y();
  int top_edge = target_top_left.x();
  int right_edge = left_edge + get_target_width();
  int bottom_edge = top_edge + get_target_height();

  // Define corners of target
  top_right = {top_edge, right_edge};
  bottom_left = {bottom_edge, left_edge};
  bottom_right = {bottom_edge, right_edge};
}

// Draw the target in isaac sight
void DepthEvaluator::drawTarget() {
  Vector<double, 2> top_right;
  Vector<double, 2> bottom_right;
  Vector<double, 2> bottom_left;

  getTargetDimensions(top_right, bottom_right, bottom_left);

  show("Region", [&](sight::Sop& sop) {
    // Draw a filled transluscent green box
    sop.style = sight::SopStyle{"rgba(0,255,0,0.5)", true};
    sop.add(geometry::RectangleD::FromOppositeCorners(target_top_left, bottom_right));
  });
  show("Marker", [&](sight::Sop& sop) {
    // Draw a black X
    sop.style = sight::SopStyle{"#000000"};
    sop.add(geometry::LineSegment2d(target_top_left, bottom_right));
    sop.add(geometry::LineSegment2d(top_right, bottom_left));
    sop.add(geometry::LineSegment2d(target_top_left, bottom_left));
    sop.add(geometry::LineSegment2d(bottom_left, bottom_right));
    sop.add(geometry::LineSegment2d(bottom_right, top_right));
    sop.add(geometry::LineSegment2d(top_right, target_top_left));
  });
}

}  // namespace evaluators
}  // namespace isaac
