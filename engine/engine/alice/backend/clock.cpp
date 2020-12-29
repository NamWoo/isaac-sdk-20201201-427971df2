/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "clock.hpp"

#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/time.hpp"

namespace isaac {
namespace alice {

void Clock::start() {
  reset();
}

void Clock::stop() {}

void Clock::reset() {
  offset_ = NowCount();
  reference_ = 0;
  advanced_ = 0;
}

void Clock::setTimeScale(double time_scale) {
  reference_ = timestamp();
  time_scale_ = time_scale;
}

int64_t Clock::timestamp() const {
  const int64_t now_raw = NowCount() + advanced_;
  const int64_t now = now_raw - offset_;
  if (time_scale_ == 1.0) {
    return now;
  }
  const int64_t result =
      reference_ + static_cast<int64_t>(time_scale_ * static_cast<double>(now - reference_));
  return result;
}

double Clock::time() const {
  return ToSeconds(timestamp());
}

// TODO : sleep based on dialated time correctly
void Clock::sleep(double duration) const {
  Sleep(SecondsToNano(duration));
}

void Clock::advance(int64_t dt) {
  advanced_ += dt;
}

}  // namespace alice
}  // namespace isaac
