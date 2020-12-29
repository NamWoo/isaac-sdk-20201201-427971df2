/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "engine/core/optional.hpp"

namespace isaac {
namespace scheduler {

// An opaque type for communicatiing with the scheduler about jobs.
// All communication will be done via handles to avoid users holding onto
// memory values.
using JobHandle = uint64_t;

// An action for the job to execute.
// Jobs do not take arguments and do not return results. All such functionality
// is assumed to be within the function body and closure.
using JobAction = std::function<void()>;

// Specifies how jobs are executed
enum class ExecutionMode : uint8_t {
  // Execute the job repeatedly and without waiting between executions.
  // This is useful for persistent activities such as repeatedly polling hardware
  // or running a background task.
  kBlocking,
  // Execute a job that is potentially blocking once.
  // This is useful for performing asynchronous work that may block such as polling hardware.
  kBlockingOneShot,
  // Execute a task once and remove from the scheduler afterwards
  kOneShotTask,
  // Execute a task periodically at fixed time slots.
  // Can skip ticks if the job takes too long to execute.
  kPeriodicTask,
  // Eecute a task only when events are received
  kEventTask
};

// This descriptor contains all the information needed to create a job within
// the system.
struct JobDescriptor {
  // This function is called when the job executes.
  JobAction action;
  // The execution group for a job
  std::string execution_group;
  // Name to use with the job for logging purposes
  std::string name;
  // If execution mode is set to kPeriod then period reprents the frequency in nanoseconds
  // IF execution mode is set to kOneShot then target_start_time reprents when the job will begin.
  union {
    int64_t period;
    int64_t target_start_time;
  };
  // Amount of time in nanoseconds the job can wait past its execution target.
  int64_t slack = 0;
  // Expected deadline for the job to complete
  std::optional<int64_t> deadline;
  // If two jobs start in the same window this priority acts as the tie breaker for scheduling.
  int priority = 0;
  // Specifies how jobs are executed
  ExecutionMode execution_mode;
  // Specifies how often this event can be queued if triggered multiple times. If set to -1 no
  // limit will be used.
  int event_trigger_limit = 0;
  // Should this job track its work statistics
  bool has_statistics = true;
  // List of groups this job belong to. Only one job of a given group can be run in the same time.
  std::vector<std::string> groups;
};

}  // namespace scheduler
}  // namespace isaac
