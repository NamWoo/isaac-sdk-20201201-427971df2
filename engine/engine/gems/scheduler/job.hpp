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
#include <limits>
#include <memory>
#include <mutex>

#include "engine/gems/scheduler/job_descriptor.hpp"
#include "engine/gems/scheduler/job_statistics.hpp"

namespace isaac {
namespace scheduler {

// A job waiting for execution
struct Job {
 public:
  Job() : running(false), scheduled(false), tombstone(false), pending(false) {
    job_lock = std::make_shared<std::mutex>();
  }

  // An execution time which will never happen; or only after mankind long perished or rules
  // an intergalactic empire spanning thousands of star systems.
  static constexpr int64_t kNever = std::numeric_limits<int64_t>::max();
  // Represents a job which is not valid or assigned
  static constexpr JobHandle kNullHandle = static_cast<JobHandle>(0);
  // Descriptor struct for a job
  JobDescriptor description;
  // The handle of the job. Used to reschedule the job and for other checks.
  JobHandle self = kNullHandle;
  // A parent job for spawned events.
  JobHandle parent = kNullHandle;
  // Mutex to protect jobs while running and to sync with waitForJobDestruction/Stop
  std::shared_ptr<std::mutex> job_lock;
  // Time at which the job should be exeucted
  int64_t target_time = kNever;
  // Indicates if a job is current executing.
  std::atomic<bool> running;
  // Indicates if a job is part of the schedule.
  std::atomic<bool> scheduled;
  // Indicates if a job has been marked for destruction.
  std::atomic<bool> tombstone;
  // Indicates the job has been queued and is pending processing
  std::atomic<bool> pending;
  // Runs the job and sets the new target time.
  void run();
};

}  // namespace scheduler
}  // namespace isaac
