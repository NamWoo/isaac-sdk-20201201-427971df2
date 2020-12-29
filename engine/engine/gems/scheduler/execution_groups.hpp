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
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/scheduler/execution_group_descriptor.hpp"
#include "engine/gems/scheduler/job_descriptor.hpp"
#include "engine/gems/scheduler/timed_job_queue.hpp"

namespace isaac {
namespace scheduler {
class TimeMachine;
class Clock;
class Job;
class JobStatistics;

// An execution group represents a group of processing cores and behaviors.
// The execution group handles the dispatching and scheduling of work
// based upon the descriptor used to create it. These groups currently
// use std::thread with some platform specific pthread behavior as needed.
class ExecutionGroup {
 public:
  explicit ExecutionGroup(const ExecutionGroupDescriptor& descriptor_, Clock* clock,
                          TimeMachine* time_machine, std::atomic<uint64_t>* handle_counter);
  // Description of this execution group
  ExecutionGroupDescriptor description;
  // Start the execution group
  void start();
  // Stop the execution group
  void stop();

  // Creates a job that can be added to the schedule. Job will not be added to
  // the execution schedule until startJob has been invoked.
  JobHandle createJob(const JobDescriptor& descriptor);
  // Destroys a job. If a job is running it will finish execution.
  // If the job relies on external state call waitForJobDestruction to
  // prevent race conditions
  void destroyJob(const JobHandle& handle);
  // Add a job to the schedule. Once a job is in the schedule it can be executed.
  void startJob(const JobHandle& handle);
  // Removes a job from the schedule. This does not destroy the job, but
  // rather stops its execution until startJob is called again.
  // If a job is running it will finish execution.
  // If the job relies on external state call waitForJobStop to
  // prevent race conditions
  void stopJob(const JobHandle& handle);

  // Waits until it is guaranteed the job will not attempt to execute or access
  // external state. i.e., until it is guaranteed it will not attempt to invoke
  // its Action again.
  void waitForJobDestruction(const JobHandle& handle);

  // Waits for a job to be flushed from the schedule. This guarantees that it will
  // be safe to modify external state related to the job.
  void waitForJobStop(const JobHandle& handle);

  // Registers a set of events which will trigger a specified job to be executable
  // when notified that such an event has occurred.
  void registerEvents(const JobHandle& handle, const std::unordered_set<std::string>& events);
  // Unregisters a job from a given set of events
  void unregisterEvents(const JobHandle& handle, const std::unordered_set<std::string>& events);
  // Notifies the scheduler that an event has occurred.
  void notify(const std::string& event, int64_t targe_time);

  // Returns a copy of the current job statistics
  JobStatistics getJobStatistics(const JobHandle& handle) const;
  // Returns a copy of the current statistics for all jobs
  std::vector<JobStatistics> getJobStatistics() const;
  // wrapper function to check if an execution group is running.
  bool isRunning() { return is_running_.load(); }
  // Returns the scheduler stats
  double getExecutionDelay() const { return execution_delay_; }

 private:
  // Implementation of createJob which assume the mutex has been locked.
  JobHandle createJobImpl(const JobDescriptor& descriptor);
  // Waits for the next job
  Job* acquireJob();
  // starts the next event if there are multiple events queued.
  void launchNextEvent(JobHandle handle);
  // Creates a worker on the specified core.
  void createWorker(int core);
  // Creatse a blocker that will work on the specified cores in the descriptor.
  void createBlocker(Job* job);
  // Stops all threads.
  void stopThreads();

  // adds or removes a job to the schedule.
  void scheduleJob(const JobHandle& handle, bool schedule);
  // adds an event to the schedule.
  void scheduleEvent(const JobHandle& handle, int64_t start_time);
  // handles job mainentance issues and places the job on the work queue for processing.
  void dispatchJob(const JobHandle& handle);
  // internal hepler function that is used to get the stats for an executing job
  JobStatistics* getStats(const JobHandle& handle);
  // Updates statistics
  void updateExecutionDelay(int64_t dt);

  // A queue which handles scheduling jobs.
  using JobQueue = TimedJobList<Job*>;
  // The clock for the execution group
  Clock* clock_;
  // A time machine for skipping time during simulation
  TimeMachine* time_machine_;
  // Each group will have its own job queue.
  JobQueue job_queue_;

  // A worker thread for executing any non blocking tasks
  struct Worker {
    // Execution thread for worker
    std::thread thread;
    // The worker main loop
    void main(ExecutionGroup* group);

   private:
    // Helper function to update the statistics for the given job.
    void updateJobStatistics(Job* job, ExecutionGroup* group, int64_t start_time, int64_t stop_time,
                             int64_t lateness_dt, int64_t execution_delay);
    // Helper function to update the jobs scheduling semantics after execution.
    void updateJobSchedule(Job* job, ExecutionGroup* group);
  };

  // A thread for executing blocking tasks.
  struct BlockingWorker {
    // Execution thread for blocking worker.
    std::thread thread;
    // The blocker main loop
    void main(ExecutionGroup* group, Job* job, bool repeat);
  };

  // List of workers used by the scheduler
  std::vector<std::unique_ptr<Worker>> workers_;
  // List of blockers launched from the scheduler
  std::vector<std::unique_ptr<BlockingWorker>> blockers_;
  // Mutex to ensure creation and destruction of threads can be synchronized
  mutable std::mutex thread_creation_mutex_;

  // If events are received before the execution group starts they will be executed immediately.
  std::unordered_set<std::string> pre_start_events_;
  // Any jobs that are started before the execution group has started will be queued.
  std::unordered_set<JobHandle> pre_start_jobs_;

  // A list of all jobs associated with this execution group
  std::unordered_map<JobHandle, std::unique_ptr<Job>> job_pool_;
  // A list of statis for each job.  Store separate from jobs so their life time is independent.
  std::unordered_map<JobHandle, std::unique_ptr<JobStatistics>> job_statistics_pool_;
  // mutex used to proteck the job and stats pools
  mutable std::mutex job_pool_mutex_;

  // For each event a set of jobs which will be triggered
  std::unordered_map<std::string, std::unordered_set<JobHandle>> event_to_jobs_;
  // The jobs currently in flight for a given event.
  std::unordered_map<JobHandle, std::unordered_set<JobHandle>> event_jobs_in_flight_;

  // Helper struct for queueing multiple events.
  struct Event {
    JobHandle job;
    int64_t start_time;
  };
  // Earliest deadline goes first.
  struct EventCmp {
    bool operator()(const Event& a, const Event& b) { return a.start_time > b.start_time; }
  };
  // These queues are used to handle multiple event dispatching by keeping
  // the next events out of the schedule queue until the current event finishes
  // execution.
  using EventQueue = std::priority_queue<Event, std::vector<Event>, EventCmp>;
  std::unordered_map<JobHandle, EventQueue> event_jobs_dispatch_queue_;

  // protects the event structures.
  mutable std::mutex events_mutex_;

  // statistics about job scheduling
  double execution_delay_;

  // Indicates if the group is currently running.
  std::atomic<bool> is_running_;

  // Pthreads specific implementation of core pinning.
  cpu_set_t cpu_set;

  // Can be used to get a new job handle.
  std::atomic<uint64_t>* handle_counter_;

  friend class BlockingWorker;
  friend class Worker;
  friend class TimeMachine;
};

}  // namespace scheduler
}  // namespace isaac
