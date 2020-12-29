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
#include <mutex>
#include <vector>

#include "engine/core/optional.hpp"
#include "engine/gems/scheduler/clock.hpp"
#include "engine/gems/scheduler/execution_groups.hpp"

namespace isaac {
namespace scheduler {

// A helper class to control the flow of time based on worker load. Currently a simple strategy is
// used which advances time each time all workers are idle. This will not work properly with jobs
// which are scheduled in "blocking" mode.
class TimeMachine {
 public:
  TimeMachine(Clock* clock)
      : clock_(clock), total_worker_count_(0), idle_count_(0), is_running_(false) {}
  // Sets the total number of workers
  void setTotalWorkerCount(unsigned total);
  // Registers a given execution group with the time machine so it can examine
  // consistent skip ahead for all groups
  void registerExecutionGroup(ExecutionGroup* group);
  // Reports a single worker as idle
  void reportIdle();
  // Reports a single worker as busy
  void reportBusy();
  // Starts the time machine and allows for skipping time.
  void start();
  // Stops the time machine and disables time skipping.
  void stop();

 private:
  // Helper function to see how far ahead the system can skip
  void checkSkip();

  Clock* clock_;
  std::mutex mutex_;
  unsigned total_worker_count_;
  std::atomic<unsigned> idle_count_;
  std::vector<ExecutionGroup*> execution_groups_;
  std::atomic<bool> is_running_;
};

}  // namespace scheduler
}  // namespace isaac
