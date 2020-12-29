/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/


#include "time_machine.hpp"

#include <algorithm>
#include <limits>

namespace isaac {
namespace scheduler {

void TimeMachine::setTotalWorkerCount(unsigned total) {
  std::unique_lock<std::mutex> lock(mutex_);
  total_worker_count_ = total;
}

void TimeMachine::registerExecutionGroup(ExecutionGroup* group) {
  execution_groups_.push_back(group);
}

void TimeMachine::reportIdle() {
  idle_count_++;
  if (idle_count_ == total_worker_count_) {
    std::unique_lock<std::mutex> lock(mutex_);
    checkSkip();
  }
}

void TimeMachine::reportBusy() {
  idle_count_--;
}

void TimeMachine::start() {
  is_running_ = true;
}

void TimeMachine::stop() {
  is_running_ = false;
}

void TimeMachine::checkSkip() {
  if (!is_running_) {
    return;
  }
  // This is 5 times the scheduling fudge factor in the timed_job_queue.
  constexpr int64_t kFudge = 500'000;
  const int64_t now = clock_->now();
  int64_t next_min = std::numeric_limits<int64_t>::max();
  for (auto* group : execution_groups_) {
    auto next = group->job_queue_.getNextTargetTime();
    if (!next) {
      continue;
    }
    next_min = std::min(next_min, *next);
  }
  if (next_min == std::numeric_limits<int64_t>::max() || idle_count_ != total_worker_count_) {
    return;
  }
  clock_->advance(std::max<int64_t>(0, next_min - now - kFudge));
  for (auto* group : execution_groups_) {
    group->job_queue_.wakeAll();
  }
}
}  // namespace scheduler
}  // namespace isaac
