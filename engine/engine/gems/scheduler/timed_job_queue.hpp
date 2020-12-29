/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <condition_variable>
#include <list>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/scheduler/clock.hpp"

namespace isaac {
namespace scheduler {

// A thread-safe queue which sorts by target execution time. It provides a blocking pop to get
// jobs precisely when the time is right.
//  T: The type of jobs to store in the queue
//  Eq: (T,T) -> bool: true if jobs are identical
template <typename JobT>
class TimedJobList {
 public:
  TimedJobList(Clock* clock) : clock_(clock), is_running_(false) {}
  // Number of queued jobs. This should only be used for logging purposes as it is not acquiring
  // a lock.
  size_t sizeUnsafe() const { return queue_.size(); }
  // Adds a job to the queue to be executed at the given target time. O(log(n))
  void insert(JobT job, int64_t target_time, int64_t slack, int priority);
  // Notify that a job has completed.
  void notifyDone(JobT job);
  // This is a blocking call which will wait for an job.
  void waitForJob(JobT&);
  // Sets the job list to a running state
  void start() { is_running_.store(true); }
  // Sets the job list to a stopped state and wakes all waiting threads to flush.
  void stop() {
    // Acquire the lock to ensure a thread isn't trying to acquire a job
    // while invoking stop.
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    // Stop the queue
    is_running_.store(false);
    // Wake all waiting threads
    queue_cv_.notify_all();
  }
  // Gets the target time of the next job; or returns nullopt if there is no job
  std::optional<int64_t> getNextTargetTime() const {
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    } else {
      return queue_.top().target_time;
    }
  }
  // wakes up one waiting threads
  void wakeOne() { queue_cv_.notify_one(); }
  // wakes all waiting threads
  void wakeAll() { queue_cv_.notify_all(); }

 private:
  // We allow a small tolerance when executing jobs
  static constexpr int64_t kTimeFudge = 100'000;  // 0.1 ms

  // Helper class used to store jobs with additional information
  struct Item {
    // The job to execute
    JobT job;
    // The target time at which the job should be executed
    int64_t target_time;
    // The amount of time the target time can slip by
    int64_t slack;
    // The priority is used as a tie breaker when two jobs would be scheduled close together.
    int priority;
  };

  // Sorts jobs such that the earliest non-running job has highest priority
  struct ItemPriorityCmp {
    bool operator()(const Item& a, const Item& b) {
      // If two jobs with their slack times are scheduled within a small
      // interval of each other check their priorities,
      // Otherwise earliest deadline goes first.
      const int64_t a_time = a.target_time + a.slack;
      const int64_t b_time = b.target_time + b.slack;

      if (std::abs(a_time - b_time) < kTimeFudge && a.priority != b.priority) {
        return a.priority < b.priority;
      } else {
        return a_time > b_time;
      }
    }
  };

  // Marks all the groups that are blocked by a given job.
  void blockGroups(const std::vector<std::string>& groups, JobT job);
  // Unblocks the groups that were blockd by a given job.
  void unblockGroups(const std::vector<std::string>& groups, JobT job);
  // Returns whether any of these groups is blocked (by another job running which belongs to one of
  // those groups).
  bool isAnyGroupRunning(const std::vector<std::string>& groups) const;

  // This function pointer should be set to utilize the desired clock for the queue.
  Clock* clock_;
  // state variable to control if threads should block on waiting for jobs
  std::atomic<bool> is_running_;

  mutable std::mutex queue_cv_mutex_;
  std::condition_variable queue_cv_;

  // List of futur events sorted by their scheduled execution time.
  std::priority_queue<Item, std::vector<Item>, ItemPriorityCmp> queue_;
  // List of events overdue, we have passed their scheduled execution time but have not been able to
  // run yet.
  std::list<Item> pending_;
  // List of groups with a job belonging to this group which is already running.
  std::unordered_map<std::string, JobT> running_groups_;
};

// -------------------------------------------------------------------------------------------------

template <typename JobT>
void TimedJobList<JobT>::insert(JobT job, int64_t target_time, int64_t slack, int priority) {
  {
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    queue_.push(Item{std::move(job), target_time, slack, priority});
  }
  queue_cv_.notify_one();
}

template <typename JobT>
bool TimedJobList<JobT>::isAnyGroupRunning(const std::vector<std::string>& groups) const {
  for (const std::string& group : groups) {
    // Check if another job from the same group is running.
    if (running_groups_.count(group) > 0) {
      return true;
    }
  }
  return false;
}

template <typename JobT>
void TimedJobList<JobT>::blockGroups(const std::vector<std::string>& groups, JobT job) {
  for (const std::string& group : groups) {
    running_groups_[group] = job;
  }
}

template <typename JobT>
void TimedJobList<JobT>::unblockGroups(const std::vector<std::string>& groups, JobT job) {
  for (const std::string& group : groups) {
    auto it = running_groups_.find(group);
    if (it != running_groups_.end() && it->second == job) {
      running_groups_.erase(it);
    }
  }
}

template <typename JobT>
void TimedJobList<JobT>::notifyDone(JobT job) {
  std::unique_lock<std::mutex> lock(queue_cv_mutex_);
  unblockGroups(job->description.groups, job);
  wakeOne();
}

template <typename JobT>
void TimedJobList<JobT>::waitForJob(JobT& job) {
  while (is_running_) {
    // Aquire the lock to check for a job
    std::unique_lock<std::mutex> lock(queue_cv_mutex_);
    // Double check the running status as a stop could have been invoked.
    if (!is_running_) {
      return;
    }
    int64_t wait_duration = 0;
    const int64_t now = clock_->now();
    while (!queue_.empty()) {
      // We have a job, check if it is time for it
      Item top_item = queue_.top();
      const int64_t eta = top_item.target_time - now;
      // If the jobs target time is within our fudge factor fire now.
      if (eta <= kTimeFudge) {
        pending_.push_back(top_item);
        queue_.pop();
      } else {
        // wait till next eta minus a small window
        wait_duration = std::max(eta - kTimeFudge, static_cast<int64_t>(0));
        break;
      }
    }
    // Loop through the list of pending jobs to find one that can be run.
    for (auto pending_job = pending_.begin(); pending_job != pending_.end(); ++pending_job) {
      if (isAnyGroupRunning(pending_job->job->description.groups)) {
        continue;
      }
      blockGroups(pending_job->job->description.groups, pending_job->job);
      job = pending_job->job;
      pending_job->target_time = now;
      pending_.erase(pending_job);
      return;
    }
    // Wait for a new job or until time is up (the lock is released while waiting)
    if (wait_duration > 0) {
      queue_cv_.wait_for(lock, std::chrono::nanoseconds(wait_duration));
    } else {
      queue_cv_.wait(lock);
    }
  }
}

}  // namespace scheduler
}  // namespace isaac
