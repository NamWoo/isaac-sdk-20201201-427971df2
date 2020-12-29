/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once
#include <atomic>

namespace isaac {
namespace alice {

// Manages time for the backends
class Clock  {
 public:
  // Starts the clock
  void start();

  // Resets the clock to zero. Should only be called at the very start of the application.
  void reset();

  // Stops the clock
  void stop();

  // Sets the current time scale. If smaller than one, time will run slower for this application. If
  // greater then one, time will run faster for this application.
  void setTimeScale(double time_scale);

  // The current time in nanoseconds
  int64_t timestamp() const;
  // The current time in seconds
  double time() const;

  // Sleeps the calling thread for a duration in seconds
  void sleep(double duration) const;

  // Advance time by a given interval
  void advance(int64_t dt);

 private:
  int64_t offset_;
  int64_t reference_;
  std::atomic<int64_t> advanced_;
  double time_scale_ = 1.0;
};

}  // namespace alice
}  // namespace isaac
