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
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/scheduler/execution_groups.hpp"
#include "engine/gems/scheduler/job_descriptor.hpp"
#include "engine/gems/scheduler/job_statistics.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace scheduler {
class Clock;
class TimeMachine;

// This class provides the main entry point and interface to the scheduler subsystem
// The actual details of scheduling are handled in the concept of execution groups
class Scheduler {
 public:
  // default worker group name.
  static constexpr char kDefaultWorkerGroup[] = "__WorkerGroup__";
  // default blocker group name.
  static constexpr char kDefaultBlockerGroup[] = "__BlockerGroup__";

  Scheduler(const std::vector<ExecutionGroupDescriptor>& execution_groups, Clock* clock);
  ~Scheduler();

  // Creates a job that can be added to the schedule. Job will not be added to
  // the execution schedule until addJob has been invoked.
  std::optional<JobHandle> createJob(const JobDescriptor& descriptor);
  // Destroys a job. If a job is running it will finish execution.
  // If the job relies on external state call waitForJobDestruction to
  // prevent race conditions
  void destroyJob(const JobHandle& handle);
  // Add a job to the schedule. Once a job is in the schedule it can be executed.
  void startJob(const JobHandle& handle) const;
  // Removes a job from the schedule. This does not destroy the job, but
  // rather stops its execution until addJob is called again.
  // If a job is running it will finish execution.
  // If the job relies on external state call waitForJobStop to
  // prevent race conditions
  void stopJob(const JobHandle& handle) const;

    // Waits until it is guaranteed the job will not attempt to execute or access
  // external state. i.e., until it is guaranteed it will not attempt to invoke
  // its Action again.
  void waitForJobDestruction(const JobHandle& handle) const;
  // Waits .for a job to be flushed from the schedule. This guarantees that it will
  // be saf.e to modify external state related to the job.
  void waitForJobStop(const JobHandle& handle) const;

  // Convenience functions to wrap common paired behaviors.
  // Equivalent to calling createJob and startJob
  std::optional<JobHandle> createJobAndStart(const JobDescriptor& descriptor);
  // Equivalent to calling stopJob and waitForJobStop
  void stopJobAndWait(const JobHandle& handle);
  // Equivalent to calling destroyjob and waitForJobDestruction
  void destroyJobAndWait(const JobHandle& handle);

  // Registers a set of events which will trigger a specified job to be executable
  // when notified that such an event has occurred.
  void registerEvents(const JobHandle& handle, const std::unordered_set<std::string>& events) const;
  // Unregisters a job from a given set of events
  void unregisterEvents(const JobHandle& handle,
                        const std::unordered_set<std::string>& events) const;
  // Notifies the scheduler that an event has occurred.
  void notify(const std::string& event, int64_t target_time) const;

  // Starts. the scheduler and begins execution.
  void start();
  // Stops the scheduler and halts execution.
  void stop();
  // Runs start, sleeps for the given duration then stops
  void startWaitStop(double duration);

  // Returns true if the scheduler is running
  bool isRunning() const { return is_running_.load(); }

  // Get job statistics
  JobStatistics getJobStatistics(const JobHandle& handle) const;
  // Returns a copy of the current statistics for all jobs
  std::vector<JobStatistics> getJobStatistics() const;
  // Get scheduling statistics
  double getExecutionDelay() const;

  // Enable the use of the time machine to compress the schedule if possible
  void enableTimeMachine();
  // Disable the use of the time machine to stop compressing time if it is running.
  void disableTimeMachine();

 private:
  // Simple helper function to get the name of a the execution group who owns a given handle.
  std::optional<std::string> findExecutionGroupName(const JobHandle& handle) const;
  // Simple helper function to get the group given a name.
  ExecutionGroup* findExecutionGroup(const std::string& group_name) const;
  // Print the list of execution groups
  void logExecutionGroups() const;
  // Sets up any default groups if cores are not fully scheduled manually.
  void createDefaultExecutionGroups();

  // Clock to drive the scheduler
  Clock* clock_;
  std::unique_ptr<TimeMachine> time_machine_;

  // Indicates if the scheduler is currently running.
  std::atomic<bool> is_running_;

  // Atomic counter global across all execution groups for creating job handle ids.
  std::atomic<uint64_t> handle_counter_;

  // Collection of execution groups for the scheduler
  mutable std::unordered_map<std::string, ExecutionGroup> execution_groups_;
  // Mapping between handles and execution groups
  std::unordered_map<JobHandle, std::string> job_to_execution_group_;
  // Lock to protect iterators under access to job_to_execution_group_
  mutable std::mutex job_map_mutex_;
};

}  // namespace scheduler
}  // namespace isaac
