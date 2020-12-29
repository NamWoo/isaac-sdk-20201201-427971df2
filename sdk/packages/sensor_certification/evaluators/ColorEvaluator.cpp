/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/sensor_certification/evaluators/ColorEvaluator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/gems/image/color.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"

#include "messages/camera.hpp"
#include "messages/image.hpp"
#include "messages/math.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace isaac {
namespace evaluators {

namespace {

constexpr float kTestTime = 1.0;
constexpr float kTimeoutSeconds = 4.0;
constexpr int kMovementSpeed = 5;

}  // namespace

void ColorEvaluator::start() {
  message_count_ = 0;
  expected_frames_ = get_frame_rate()*kTestTime;
  color_image_saved_ = false;
  colors[0] = "RED";
  colors[1] = "GREEN";
  colors[2] = "BLUE";

  // initialize the target location at the center of the image
  int left_edge = get_image_width()/2 - get_target_width() / 2;
  int top_edge = get_image_height()/2 - get_target_height()/2;
  target_top_left = {top_edge, left_edge};

  drawTarget();
  tickPeriodically();
}

void ColorEvaluator::tick() {
  if (get_target_cmd() != 5) {
    // If user is still adjusting target and setup then clear depth messages
    rx_color_listener().processAllNewMessages([this] (auto proto, int64_t pubtime,
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

  rx_color_listener().processAllNewMessages([this] (auto proto, int64_t pubtime,
                                                    int64_t acqtime) {
    // Convert ImageProto to opencv matrix
    message_count_++;

    ImageConstView3ub input_image;
    FromProto(proto, rx_color_listener().buffers(), input_image);

    const size_t rows = input_image.rows();
    const size_t cols = input_image.cols();

    const cv::Mat image =
      cv::Mat(rows, cols, CV_8UC3,
              const_cast<void*>(static_cast<const void*>(input_image.data().pointer())));

    // Crop image to ROI
    const int crop_rows = get_target_height();
    const int crop_cols = get_target_width();

    cv::Mat crop = cv::Mat(crop_rows, crop_cols, CV_8UC3);
    cv::Rect target_rect(target_top_left.y(), target_top_left.x(), crop_cols, crop_rows);
    crop = image(target_rect).clone();

    // Calculate mean pixel values of cropped image
    cv::Scalar mean = cv::mean(crop);

    // Determine color based on which of R, G, or B values are highest
    std::vector<double> rgb_vals = {mean[0], mean[1], mean[2]};
    max_idx = std::max_element(rgb_vals.begin(), rgb_vals.end()) - rgb_vals.begin();

    if (max_idx != get_target_color()) {
      reportMessage("Frame %u/%u: incorrect color detected in box. Expected %s and got %s\n",
                    message_count_,
                    expected_frames_,
                    colors[get_target_color()],
                    colors[max_idx]);
      reportFailure();
    }
  });

  if (message_count_ >= expected_frames_) {
    reportMessage("Detected color is %s\n", colors[max_idx]);
    reportSuccess();
    return;
  }

  checkTimeout();
}

float ColorEvaluator::getTimeoutSeconds() {
  return kTimeoutSeconds;
}

void ColorEvaluator::reportTimeoutMessage() {
  reportMessage(
    "Timed out after %u seconds. Only received %u/%u expected frames\n",
    (unsigned int)kTimeoutSeconds,
    message_count_,
    expected_frames_);
}

void ColorEvaluator::saveColorImage(const ImageConstView3ub& image) {
  std::string name = get_report_directory() + "/color.png";
  isaac::SavePng(image, name);
}

// Calculates remaining corners of target based on top left class variable
void ColorEvaluator::getTargetDimensions(Vector<double, 2>& top_right,
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
void ColorEvaluator::drawTarget() {
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
