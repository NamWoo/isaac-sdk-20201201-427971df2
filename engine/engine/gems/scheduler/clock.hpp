/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>

namespace isaac {
namespace scheduler {

// Clock to drive the scheduler.  Users can override the clock for custom behaviors.
class Clock {
 public:
  virtual ~Clock() = default;
  // Reads the current time in nano seconds off the clock
  virtual int64_t now() = 0;
  // Advances the current time by dt amount
  virtual void advance(int64_t dt) = 0;
};

}  // namespace scheduler
}  // namespace isaac
