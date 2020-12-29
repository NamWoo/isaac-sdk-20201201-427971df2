/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "execution_groups.hpp"

#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cinttypes>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "engine/gems/scheduler/clock.hpp"
#include "engine/gems/scheduler/job.hpp"
#include "engine/gems/scheduler/job_statistics.hpp"
#include "engine/gems/scheduler/time_machine.hpp"
#include "nvToolsExt.h"  //NOLINT

namespace isaac {
namespace scheduler {

namespace {
// Decay factor for execution statistics TODO: normalize to ticks/s
constexpr double kExecutionDelayDecay = 0.001;
}  // namespace

ExecutionGroup::ExecutionGroup(const ExecutionGroupDescriptor& descriptor_, Clock* clock,
                               TimeMachine* time_machine, std::atomic<uint64_t>* handle_counter)
    : description(descriptor_),
      clock_(clock),
      time_machine_(time_machine),
      job_queue_(clock),
      is_running_(false),
      handle_counter_(handle_counter) {
  // create a cpu set for blocking workers in case this group uses them.
  CPU_ZERO(&cpu_set);
  for (auto core : description.cores) {
    CPU_SET(core, &cpu_set);
  }
}

JobHandle ExecutionGroup::createJobImpl(const JobDescriptor& descriptor) {
  JobHandle handle = ++(*handle_counter_);
  // Add job to pool and add a slot to track its statistics
  job_pool_[handle] = std::make_unique<Job>();
  if (descriptor.has_statistics) {
    job_statistics_pool_[handle] = std::make_unique<JobStatistics>();
    job_statistics_pool_[handle]->descriptor = descriptor;
  }
  Job* job = job_pool_[handle].get();
  job->self = handle;
  job->description = descriptor;
  return handle;
}

JobHandle ExecutionGroup::createJob(const JobDescriptor& descriptor) {
  std::lock_guard<std::mutex> lock(job_pool_mutex_);
  return createJobImpl(descriptor);
}

void ExecutionGroup::destroyJob(const JobHandle& handle) {
  // Local copy of events to be waited on if needed.
  std::unordered_set<JobHandle> events_in_flight;
  {
    std::lock_guard<std::mutex> lock(job_pool_mutex_);
    // If no such job exists do nothing.
    auto it = job_pool_.find(handle);
    if (it == job_pool_.end()) {
      return;
    }
    Job* job = it->second.get();
    // Mark Job as dead. Clean up will happen on next attempt to execute.
    job->tombstone.store(true);

    // If the scheduler wasn't running yet delete the job directly. Events are handled separately.
    if (!is_running_ && job->description.execution_mode != ExecutionMode::kEventTask) {
      job_pool_.erase(handle);
      pre_start_jobs_.erase(handle);
    }

    // Event jobs have more complicated book keeping and
    // since they never directly execute need to be deleted manually.
    if (job->description.execution_mode == ExecutionMode::kEventTask) {
      // Remove job from events.
      {
        std::lock_guard<std::mutex> event_lock(events_mutex_);
        for (auto& kvp : event_to_jobs_) {
          kvp.second.erase(handle);
        }
      }
      events_in_flight = event_jobs_in_flight_[job->self];
      // Mark all out standing jobs as dead.
      for (auto& event_handle : events_in_flight) {
        auto job_it = job_pool_.find(event_handle);
        if (job_it == job_pool_.end()) {
          continue;
        }
        job_it->second->tombstone.store(true);
      }
      // Delete the event.
      job_pool_.erase(handle);
    }
  }
  // Wait on any outstanding events to be safe
  for (auto& handle : events_in_flight) {
    waitForJobDestruction(handle);
  }
}

void ExecutionGroup::startJob(const JobHandle& handle) {
  if (is_running_) {
    scheduleJob(handle, true);
  } else {
    pre_start_jobs_.insert(handle);
  }
}

void ExecutionGroup::stopJob(const JobHandle& handle) {
  scheduleJob(handle, false);
}

void ExecutionGroup::registerEvents(const JobHandle& handle,
                                    const std::unordered_set<std::string>& events) {
  std::lock_guard<std::mutex> lock(events_mutex_);
  // Insert the job to the specified events;
  for (const std::string& event : events) {
    event_to_jobs_[event].insert(handle);
  }
}

// Unregisters a job from a given set of events
void ExecutionGroup::unregisterEvents(const JobHandle& handle,
                                      const std::unordered_set<std::string>& events) {
  std::lock_guard<std::mutex> lock(events_mutex_);
  // Remove the job from the specified events.
  for (const std::string& event : events) {
    auto kvp = event_to_jobs_.find(event);
    if (kvp != event_to_jobs_.end()) {
      kvp->second.erase(handle);
    }
  }
}

// Notify the scheduler that an event has occurred.
void ExecutionGroup::notify(const std::string& event, int64_t target_time) {
  std::unordered_set<JobHandle> working_copy;
  {
    std::lock_guard<std::mutex> lock(events_mutex_);
    // Buffer events to be fired once the scheduler begins to run.
    if (!is_running_) {
      pre_start_events_.insert(event);
      return;
    }
    auto it = event_to_jobs_.find(event);
    if (it == event_to_jobs_.end()) {
      // No one is listening to this event
      return;
    }
    // Copy event list so the lock can be released.
    working_copy = it->second;
  }
  // Fire Events
  for (const auto& job_handle : working_copy) {
    scheduleEvent(job_handle, target_time);
  }
}

JobStatistics ExecutionGroup::getJobStatistics(const JobHandle& handle) const {
  std::lock_guard<std::mutex> lock(job_pool_mutex_);
  auto it = job_statistics_pool_.find(handle);
  if (it == job_statistics_pool_.end()) {
    LOG_WARNING(
        "Attempting to get statistics for an unknown job: Handle '%ull' | Execution Group '%s'",
        handle, description.name.c_str());
    return JobStatistics{};
  }
  return *(it->second.get());
}

std::vector<JobStatistics> ExecutionGroup::getJobStatistics() const {
  std::lock_guard<std::mutex> lock(job_pool_mutex_);
  std::vector<JobStatistics> stats;
  for (const auto& kvp : job_statistics_pool_) {
    stats.push_back(*kvp.second);
  }
  return stats;
}

void ExecutionGroup::waitForJobDestruction(const JobHandle& handle) {
  int rounds = 0;
  while (true) {
    Job* job = nullptr;
    {
      std::lock_guard<std::mutex> lock(job_pool_mutex_);
      auto it = job_pool_.find(handle);
      if (it == job_pool_.end()) {
        return;
      }
      job = it->second.get();
      if (job->description.execution_mode != ExecutionMode::kBlocking &&
          job->description.execution_mode != ExecutionMode::kBlockingOneShot) {
        // If the job is marked dead and we control the lock it is safe to return
        std::unique_lock<std::mutex> job_lock(*(job->job_lock), std::defer_lock);
        if (job_lock.try_lock()) {
          if (job->tombstone.load()) {
            return;
          }
        }
      }
    }
    rounds++;
    if (rounds % 300 == 0) {
      LOG_WARNING("waitForJobDestruction: Job: Name '%s' | Handle '%" PRIu64 "' is not stopping...",
                  job->description.name.c_str(), handle);
    }
    Sleep(10'000'000);
  }
}

void ExecutionGroup::waitForJobStop(const JobHandle& handle) {
  int rounds = 0;
  while (true) {
    Job* job = nullptr;
    {
      std::lock_guard<std::mutex> lock(job_pool_mutex_);
      auto it = job_pool_.find(handle);
      if (it == job_pool_.end()) {
        return;
      }
      job = it->second.get();
      // If the job is marked unscheduled and we control the lock it is safe to return
      std::unique_lock<std::mutex> job_lock(*(job->job_lock), std::defer_lock);
      if (job_lock.try_lock()) {
        if (!job->pending.load() && !job->scheduled.load()) {
          return;
        }
      }
    }
    rounds++;
    if (rounds % 100 == 0) {
      LOG_WARNING("waitForJobStop: Job: Name '%s' | Handle '%" PRIu64 "' is not stopping...",
                  job->description.name.c_str(), handle);
    }
    Sleep(10'000'000);
  }
}

void ExecutionGroup::scheduleJob(const JobHandle& handle, bool schedule) {
  // Local copy of events to be waited on if needed.
  std::unordered_set<JobHandle> events_in_flight;
  {
    // Lock the job pool during scheduling.
    std::lock_guard<std::mutex> lock(job_pool_mutex_);
    // Find the job
    auto it = job_pool_.find(handle);
    if (it == job_pool_.end()) {
      return;
    }
    Job* job = it->second.get();
    job->scheduled.store(schedule);
    // Event jobs have more complcated book keeping and
    // since they never directly execute need to be unscheduled manually.
    if (!schedule && job->description.execution_mode == ExecutionMode::kEventTask) {
      events_in_flight = event_jobs_in_flight_[job->self];
      // Mark all out standing jobs as dead.
      for (auto& event_handle : events_in_flight) {
        auto job_it = job_pool_.find(event_handle);
        if (job_it == job_pool_.end()) {
          continue;
        }
        job_it->second->scheduled.store(false);
      }
    }
  }
  // Wait on any outstanding events to be safe
  for (auto& event_handle : events_in_flight) {
    waitForJobStop(event_handle);
  }

  if (schedule) {
    dispatchJob(handle);
  }
}

void ExecutionGroup::scheduleEvent(const JobHandle& handle, int64_t start_time) {
  JobDescriptor event_descriptor;
  JobHandle event;
  bool start_event = false;
  {
    std::lock_guard<std::mutex> lock(job_pool_mutex_);
    Job* job = nullptr;
    auto job_it = job_pool_.find(handle);
    if (job_it != job_pool_.end()) {
      job = job_it->second.get();
    } else {
      return;
    }

    if (job->scheduled.load() == false) {
      return;
    }

    // Make sure the job is not queued more often than the allowed limit
    if (job->description.event_trigger_limit >= 0 &&
        event_jobs_in_flight_[job->self].size() >
            static_cast<size_t>(job->description.event_trigger_limit)) {
      return;
    }

    // Make a copy of the job as a one shot event.  Put this job into the queue
    // and add tracking information for other book keeping.
    event_descriptor.execution_mode = ExecutionMode::kOneShotTask;
    event_descriptor.execution_group = job->description.execution_group;
    event_descriptor.target_start_time = start_time;
    event_descriptor.action = job->description.action;
    event_descriptor.slack = job->description.slack;
    event_descriptor.name = job->description.name;
    event_descriptor.groups = job->description.groups;
    // Stats will be tracked on parent
    event_descriptor.has_statistics = false;

    event = createJobImpl(event_descriptor);

    // To ensure certain behaviors with regards to not over dispatching events
    // it is now time to do a little bit of awful copying of data.
    auto it = job_pool_.find(event);
    ASSERT(it != job_pool_.end(), "createJobImpl was supposed to create the job.");
    Job* parent = job;
    Job* child = it->second.get();
    child->parent = parent->self;
    child->job_lock = parent->job_lock;
    event_jobs_in_flight_[parent->self].insert(child->self);
    // Use the event parent pending slot to track if a job needs to be dispatched.
    if (!parent->pending.load()) {
      parent->pending.store(true);
      start_event = true;
    } else {
      event_jobs_dispatch_queue_[parent->self].push(
          {child->self, child->description.target_start_time});
    }
  }
  // If needed start the dispatch chain.
  if (start_event) {
    startJob(event);
  }
}

void ExecutionGroup::dispatchJob(const JobHandle& handle) {
  // Lock the job pool during scheduling.
  std::unique_lock<std::mutex> lock(job_pool_mutex_);
  // Find the job
  auto it = job_pool_.find(handle);
  if (it == job_pool_.end()) {
    return;
  }
  Job* job = it->second.get();
  // Cleanup the groups to allow other codelet to run.
  job_queue_.notifyDone(job);
  // Remove dead jobs from the system.
  if (job->tombstone) {
    // if the job is dead and has a parent it may be an event.
    // if it is an event then there is some clean up that needs to occur in case
    // there are pending events.
    JobHandle nextEvent;
    bool start_job = false;
    // There doesn't seem to be a faster/better way to detect an invalid handle.
    if (job->parent != static_cast<uint64_t>(0)) {
      auto it = event_jobs_in_flight_.find(job->parent);
      if (it != event_jobs_in_flight_.end()) {
        // Remove job from pending jobs in flight list.
        it->second.erase(job->self);
        // If there are other jobs pending they must be processed.
        if (it->second.size() == 0) {
          // No further processing mark job as no longer pending.
          auto parent_it = job_pool_.find(job->parent);
          if (parent_it != job_pool_.end()) {
            parent_it->second.get()->pending.store(false);
          }
        } else {
          // Dispatch the next event if needed.
          auto dispatch_queue_it = event_jobs_dispatch_queue_.find(job->parent);
          if (dispatch_queue_it != event_jobs_dispatch_queue_.end()) {
            nextEvent = dispatch_queue_it->second.top().job;
            dispatch_queue_it->second.pop();
            start_job = true;
          }
        }
      }
    }
    job_pool_.erase(it);

    if (start_job) {
      lock.unlock();
      startJob(nextEvent);
    }
    return;
  }

  // Unscheduled jobs can not be dispatched
  if (!job->scheduled) {
    return;
  }

  // Determine execution group
  switch (job->description.execution_mode) {
    case ExecutionMode::kPeriodicTask:
      // Periodic jobs that have never been run will start asap
      if (job->target_time == Job::kNever) {
        job->target_time = clock_->now();
      }
      if (!description.has_workers) {
        LOG_ERROR("Attempting to execute periodic task on group without workers");
        return;
      }
      break;

    case ExecutionMode::kOneShotTask:
      ASSERT(job->target_time == Job::kNever, "Attempting double dispatch on one shot job");
      job->target_time = job->description.target_start_time;
      if (!description.has_workers) {
        LOG_ERROR("Attempting to execute one shot task on group without workers");
        return;
      }
      break;

    // If a job is blocking it will be farmed out to a dedicated worker thread.
    case ExecutionMode::kBlocking:
    case ExecutionMode::kBlockingOneShot:
      if (description.has_workers) {
        LOG_WARNING("Executing blocking task on worker group. This may impact performance");
      }
      createBlocker(job);
      return;

    // Events are never dispatched. Only copies are dispatched.
    case ExecutionMode::kEventTask:
      return;

    default:
      PANIC("Unknown execution mode");
      return;
  }

  // Dispatch the job
  job->pending.store(true);
  job_queue_.insert(job, job->target_time, job->description.slack, job->description.priority);
}

void ExecutionGroup::updateExecutionDelay(int64_t dt) {
  const double q = static_cast<double>(dt) / 1'000'000;
  execution_delay_ = (1.0 - kExecutionDelayDecay) * execution_delay_ + kExecutionDelayDecay * q;
}

void ExecutionGroup::start() {
  is_running_ = true;
  job_queue_.start();
  if (description.has_workers) {
    for (auto core : description.cores) {
      createWorker(core);
    }
  }
  // Now that the execution groups are active add all pending jobs
  LOG_DEBUG("Launching %zd pre-start job(s)", pre_start_jobs_.size());
  for (const auto& handle : pre_start_jobs_) {
    startJob(handle);
  }
  pre_start_jobs_.clear();

  // replay events which where received before the scheduler started. This will potentially
  // schedule some of the event-based jobs.
  {
    LOG_DEBUG("Replaying %zd pre-start event(s)", pre_start_events_.size());
    for (const auto& event : pre_start_events_) {
      notify(event, clock_->now());
    }
    pre_start_events_.clear();
  }
}

void ExecutionGroup::stop() {
  if (!is_running_.exchange(false)) {
    return;
  }
  // Mark all jobs as dead.
  {
    std::lock_guard<std::mutex> lock(job_pool_mutex_);
    for (auto& kvp : job_pool_) {
      kvp.second->tombstone.store(true);
    }
  }
  // Stop all workers and thus stop execution jobs.
  LOG_DEBUG("Stopping all threads for execution group %s...", description.name.c_str());
  job_queue_.stop();
  stopThreads();

  LOG_DEBUG("Stopping all threads DONE");
  // Remove all jobs
  {
    std::lock_guard<std::mutex> lock(job_pool_mutex_);
    job_pool_.clear();
    job_statistics_pool_.clear();
  }
}

JobStatistics* ExecutionGroup::getStats(const JobHandle& handle) {
  std::lock_guard<std::mutex> lock(job_pool_mutex_);
  auto it = job_statistics_pool_.find(handle);
  if (it == job_statistics_pool_.end()) {
    return nullptr;
  }
  return it->second.get();
}

void ExecutionGroup::createWorker(int core) {
  std::lock_guard<std::mutex> lock(thread_creation_mutex_);
  cpu_set_t worker_core;
  CPU_ZERO(&worker_core);
  CPU_SET(core, &worker_core);
  workers_.emplace_back(std::make_unique<ExecutionGroup::Worker>());
  workers_.back()->thread = std::thread([=] { workers_.back()->main(this); });
  pthread_setaffinity_np(workers_.back()->thread.native_handle(), sizeof(cpu_set_t), &worker_core);
}

// TODO: Blocker threads should in theory be launched from a thread inside the blocker group.
void ExecutionGroup::createBlocker(Job* job) {
  std::lock_guard<std::mutex> lock(thread_creation_mutex_);
  blockers_.emplace_back(std::make_unique<ExecutionGroup::BlockingWorker>());
  blockers_.back()->thread = std::thread([=] {
    blockers_.back()->main(this, job, job->description.execution_mode == ExecutionMode::kBlocking);
  });
  pthread_setaffinity_np(blockers_.back()->thread.native_handle(), sizeof(cpu_set_t), &cpu_set);
}

void ExecutionGroup::stopThreads() {
  std::lock_guard<std::mutex> lock(thread_creation_mutex_);
  for (auto& worker : workers_) {
    worker->thread.join();
  }
  workers_.clear();

  for (auto& blocker : blockers_) {
    blocker->thread.join();
  }
  blockers_.clear();
}

Job* ExecutionGroup::acquireJob() {
  Job* job = nullptr;
  time_machine_->reportIdle();
  job_queue_.waitForJob(job);
  time_machine_->reportBusy();
  return job;
}

void ExecutionGroup::Worker::main(ExecutionGroup* group) {
  const std::string thread_name = group->description.name + ":WorkerThread";
  nvtxNameOsThread(syscall(SYS_gettid), thread_name.c_str());

  nvtxEventAttributes_t eventAttrib = {0};
  eventAttrib.version = NVTX_VERSION;
  eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = 0xFF76b900;  // NVIDIA GREEN
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;

  while (group->isRunning()) {
    // wait for a job
    Job* job = group->acquireJob();
    if (job != nullptr) {
      int64_t start_time = 0;
      int64_t stop_time = 0;
      int64_t overrun_dt = 0;
      int64_t execution_delay = 0;
      {
        // Do not modify the job while it is in flight.
        std::lock_guard<std::mutex> job_lock(*(job->job_lock));

        if (job->scheduled && !job->tombstone) {
          // statistics if we where late or early
          start_time = group->clock_->now();
          eventAttrib.message.ascii = job->description.name.c_str();
          nvtxRangePushEx(&eventAttrib);
          job->run();
          nvtxRangePop();
          stop_time = group->clock_->now();
          execution_delay = start_time - job->target_time;
          group->updateExecutionDelay(execution_delay);
          job->pending.store(false);
        }

        updateJobSchedule(job, group);
        overrun_dt = job->description.deadline
                         ? (stop_time - start_time) - *(job->description.deadline)
                         : 0.0;
      }
      updateJobStatistics(job, group, start_time, stop_time, overrun_dt, execution_delay);

      // Try to schedule the job again. this will trigger a clean up if needed.
      group->dispatchJob(job->self);
    }
  }
}

void ExecutionGroup::Worker::updateJobStatistics(Job* job, ExecutionGroup* group,
                                                 int64_t start_time, int64_t stop_time,
                                                 int64_t overrun_dt, int64_t execution_delay) {
  // Get the starts for the given job. If a oneshot job has a parent
  // then the parents stats will be used.
  JobHandle handle = (job->parent != Job::kNullHandle &&
                      job->description.execution_mode == ExecutionMode::kOneShotTask)
                         ? job->parent
                         : job->self;
  JobStatistics* stats = group->getStats(handle);
  // Gather data for computing statistics

  const double start_time_sec = ToSeconds(start_time);
  bool update_load = true;
  if (stats) {
    stats->num_executed++;
    if (stats->last_stop_time == 0) {
      stats->last_stop_time = start_time;
      update_load = false;
    }
    stats->total_time += stop_time - start_time;
    stats->current_rate.add(1.0, start_time_sec);
    stats->total_idle += std::max(start_time - stats->last_stop_time, static_cast<int64_t>(0));

    stats->last_stop_time = stop_time;
    const int64_t dt_busy = stop_time - start_time;
    const double dt_busy_sec = ToSeconds(dt_busy);
    stats->execution_time_median.add(dt_busy_sec);
    if (update_load) {
      stats->exec_dt.add(dt_busy_sec, start_time_sec);
      stats->current_load.add(dt_busy_sec, start_time_sec);
    }
    stats->execution_delay.add(ToSeconds(execution_delay), start_time_sec);
    if (overrun_dt > 0) {
      // The task took longer for the tick than the period
      stats->num_overrun++;
      stats->total_time_overrun += overrun_dt;
    }
  }
}

void ExecutionGroup::Worker::updateJobSchedule(Job* job, ExecutionGroup* group) {
  switch (job->description.execution_mode) {
    case ExecutionMode::kOneShotTask:
      job->tombstone.store(true);
      break;
    case ExecutionMode::kPeriodicTask: {
      job->target_time += job->description.period;
      if (group->clock_->now() - job->target_time > 0) {
        // Boost Job target time to now to try and compensate for delays.
        // TODO: Is this the best policy for this behavior?
        job->target_time = group->clock_->now();
      }
    } break;
    case ExecutionMode::kBlocking:
    case ExecutionMode::kBlockingOneShot:
      ASSERT(false, "Blocking jobs should not be on Workers");
      break;
    case ExecutionMode::kEventTask:
      ASSERT(false, "Event jobs should not be on Workers");
      break;
    default:
      PANIC("Unknown execution mode");
      break;
  }
}

void ExecutionGroup::BlockingWorker::main(ExecutionGroup* group, Job* job, bool repeat) {
  const std::string thread_name = group->description.name + ":BlockerThread";
  nvtxNameOsThread(syscall(SYS_gettid), thread_name.c_str());

  nvtxEventAttributes_t eventAttrib = {0};
  eventAttrib.version = NVTX_VERSION;
  eventAttrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
  eventAttrib.colorType = NVTX_COLOR_ARGB;
  eventAttrib.color = 0xFF76b900;  // NVIDIA GREEN
  eventAttrib.messageType = NVTX_MESSAGE_TYPE_ASCII;

  while (group->isRunning() && job->scheduled && !job->tombstone) {
    int64_t start_time = group->clock_->now();
    eventAttrib.message.ascii = job->description.name.c_str();
    nvtxRangePushEx(&eventAttrib);
    job->run();
    nvtxRangePop();
    int64_t stop_time = group->clock_->now();
    JobStatistics* stats = group->getStats(job->self);
    if (stats) {
      stats->num_executed++;
      const int64_t dt_busy = stop_time - start_time;
      stats->total_time += dt_busy;
      const double dt_busy_sec = ToSeconds(dt_busy);
      stats->execution_time_median.add(dt_busy_sec);
      const double start_time_sec = ToSeconds(start_time);
      stats->current_rate.add(1.0, start_time_sec);
      stats->current_load.add(dt_busy_sec, start_time_sec);
      stats->exec_dt.add(dt_busy_sec, start_time_sec);
    }
    if (!repeat) {
      job->tombstone = true;
      break;
    }
  }
  // Pass the job back to the scheduler to do clean up.
  group->dispatchJob(job->self);
}

}  // namespace scheduler
}  // namespace isaac
