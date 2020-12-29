/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once
#include <string>

#include "engine/alice/backend/clock.hpp"
#include "engine/core/time.hpp"

namespace isaac {
namespace alice {

// Class to implement stop watch functionality for a given clock
class Stopwatch {
 public:
  Stopwatch() : clock_(nullptr), start_(0), stop_(0), is_running_(false) {}

  // set the clock to be used by the stopwatch
  void setClock(alice::Clock* clock) { clock_ = clock; }

  // Starts tracking the time and clears the old time from the watch
  void start() {
    start_ = clock_->timestamp();
    stop_ = 0;
    is_running_ = true;
  }

  // Gets the current duration on the stopwatch in seconds or returns the measured time if
  // it has been stopped.
  double read() { return ToSeconds((stop_ == 0 ? clock_->timestamp() : stop_) - start_); }

  // stops the watch and returns the elapsed time in seconds.
  double stop() {
    stop_ = clock_->timestamp();
    is_running_ = false;
    return ToSeconds(stop_ - start_);
  }

  // Returns whether or not the stop watch is currently running
  bool running() { return is_running_; }

  // Returns whether the stopwatch is usable
  bool valid() { return clock_ != nullptr; }

  // Interval helper function.
  // If the watch is not running it will start the watch.
  // If the watch is running it will check to see if it has been running longer than dt.
  // If the dt is exceeded it will return true and restart the clock.
  bool interval(double dt) {
    if (!running()) {
      start();
    } else {
      if (dt <= read()) {
        start();
        return true;
      }
    }
    return false;
  }

 private:
  // The clock the stopwatch will use for timing
  Clock* clock_;
  // Time the watch was started at
  int64_t start_;
  // Time the watch was stopped at
  int64_t stop_;
  // Indicates if the watch is running
  bool is_running_;
};

}  // namespace alice
}  // namespace isaac
