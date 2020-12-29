/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/sensor_certification/evaluators/FrameRateEvaluator.hpp"

#include <string>

#include "engine/gems/image/color.hpp"
#include "engine/gems/image/utils.hpp"
#include "messages/camera.hpp"

namespace isaac {
namespace evaluators {

void FrameRateEvaluator::start() {
  expected_frames_ = get_test_duration() * get_target_fps();
  timeout_seconds_ = get_test_duration() * 4;
  message_count_ = 0;
  message_times_.resize(expected_frames_);
  tickPeriodically();
}

void FrameRateEvaluator::tick() {
  rx_depth_listener().processAllNewMessages([this] (auto proto, int64_t pubtime, int64_t acqtime) {
    if (get_test_depth()) {
      message_times_[message_count_++] = getTickTimestamp();
    }
  });
  rx_color_listener().processAllNewMessages([this] (auto proto, int64_t pubtime, int64_t acqtime) {
    if (!get_test_depth()) {
      message_times_[message_count_++] = getTickTimestamp();
    }
  });

  if (message_count_ >= expected_frames_) {
    calculateFrameRate();
  } else {
    checkTimeout();
  }
}

float FrameRateEvaluator::getTimeoutSeconds() {
  return timeout_seconds_;
}

void FrameRateEvaluator::reportTimeoutMessage() {
  reportMessage(
    "Timed out after %u seconds. Only received %u/%u expected frames\n",
    (unsigned int) timeout_seconds_,
    message_count_,
    expected_frames_);
}

void FrameRateEvaluator::calculateFrameRate() {
  float target_fps = get_target_fps();
  float fps = 1e9 / ((message_times_[message_count_-1] - message_times_[0]) / (message_count_ - 1));
  float error = std::abs(fps - target_fps) / target_fps;

  reportMessage(
    "Frame Rate Result:\n"
    " - Target frame rate  %02.2f fps\n"
    " - Frame rate   %02.2f fps\n"
    " - FPS error    %02.2f%%\n",
    target_fps,
    fps,
    error * 100);

  float fps_tolerance = get_fps_tolerance() / 100;
  if (error <= fps_tolerance) {
    reportSuccess();
  } else {
    reportFailure();
  }
}

}  // namespace evaluators
}  // namespace isaac
