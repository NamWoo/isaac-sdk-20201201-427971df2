/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/scheduler/scheduler.hpp"

#include <atomic>

#include "engine/core/time.hpp"
#include "engine/gems/scheduler/clock.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace scheduler {

class SchedulerClock : public scheduler::Clock {
 public:
  int64_t now() override { return NowCount(); }
  void advance(int64_t dt) override {}
};

TEST(Scheduler, TestRunTime) {
  constexpr double kDuration = 0.35;
  std::vector<ExecutionGroupDescriptor> execution_groups;
  auto clock = std::make_unique<SchedulerClock>();
  Scheduler scheduler(execution_groups, clock.get());
  const int64_t t1 = NowCount();
  scheduler.startWaitStop(kDuration);
  const int64_t t2 = NowCount();
  EXPECT_NEAR(ToSeconds(t2 - t1), kDuration, 0.05);
}

TEST(Scheduler, Periodic) {
  int count = 0;
  std::vector<ExecutionGroupDescriptor> execution_groups;
  auto clock = std::make_unique<SchedulerClock>();
  Scheduler scheduler(execution_groups, clock.get());
  JobDescriptor job_descriptor;
  job_descriptor.execution_group = "";
  job_descriptor.period = SecondsToNano(0.10);
  job_descriptor.priority = 0;
  job_descriptor.slack = 0;
  job_descriptor.execution_mode = ExecutionMode::kPeriodicTask;
  job_descriptor.name = "periodic";
  job_descriptor.action = [&] {
    Sleep(SecondsToNano(0.03));
    count++;
  };
  std::optional<JobHandle> handle = scheduler.createJob(job_descriptor);
  if (handle) {
    scheduler.startJob(*handle);
    scheduler.startWaitStop(0.55);
  }
  EXPECT_EQ(count, 6);
}

TEST(Scheduler, EventBased) {
  int count = 0;
  std::vector<ExecutionGroupDescriptor> execution_groups;
  auto clock = std::make_unique<SchedulerClock>();
  Scheduler scheduler(execution_groups, clock.get());
  JobDescriptor job_descriptor1;
  job_descriptor1.execution_group = "";
  job_descriptor1.period = SecondsToNano(0.10);
  job_descriptor1.priority = 0;
  job_descriptor1.slack = 0;
  job_descriptor1.execution_mode = ExecutionMode::kPeriodicTask;
  job_descriptor1.name = "periodic";
  job_descriptor1.action = [&] {
    Sleep(SecondsToNano(0.03));
    scheduler.notify("!", clock->now());
  };

  JobDescriptor job_descriptor2;
  job_descriptor2.execution_group = "";
  job_descriptor2.period = SecondsToNano(0.10);
  job_descriptor2.priority = 0;
  job_descriptor2.slack = 0;
  job_descriptor2.execution_mode = ExecutionMode::kEventTask;
  job_descriptor2.event_trigger_limit = 1;
  job_descriptor2.name = "event";
  job_descriptor2.action = [&] {
    Sleep(SecondsToNano(0.08));
    count++;
  };
  std::optional<JobHandle> handle1 = scheduler.createJob(job_descriptor1);
  std::optional<JobHandle> handle2 = scheduler.createJob(job_descriptor2);
  if (handle1 && handle2) {
    scheduler.registerEvents(*handle2, {"!"});
    scheduler.startJob(*handle1);
    scheduler.startJob(*handle2);
    scheduler.startWaitStop(0.55);
  }
  EXPECT_EQ(count, 6);
}

TEST(Scheduler, OneShot) {
  bool triggered = false;
  std::vector<ExecutionGroupDescriptor> execution_groups;
  auto clock = std::make_unique<SchedulerClock>();
  Scheduler scheduler(execution_groups, clock.get());
  JobDescriptor job_descriptor;
  job_descriptor.execution_group = "";
  job_descriptor.target_start_time = NowCount();
  job_descriptor.priority = 0;
  job_descriptor.slack = 0;
  job_descriptor.execution_mode = ExecutionMode::kOneShotTask;
  job_descriptor.name = "one shot";
  job_descriptor.action = [&] { triggered = true; };
  std::optional<JobHandle> handle = scheduler.createJob(job_descriptor);
  if (handle) {
    scheduler.startJob(*handle);
    scheduler.startWaitStop(0.25);
  }
  EXPECT_TRUE(triggered);
}

TEST(Scheduler, OneShot2) {
  bool triggered = false;
  std::vector<ExecutionGroupDescriptor> execution_groups;
  auto clock = std::make_unique<SchedulerClock>();
  Scheduler scheduler(execution_groups, clock.get());
  JobDescriptor job_descriptor;
  job_descriptor.execution_group = "";
  job_descriptor.target_start_time = NowCount() + SecondsToNano(0.50);
  job_descriptor.priority = 0;
  job_descriptor.slack = 0;
  job_descriptor.execution_mode = ExecutionMode::kOneShotTask;
  job_descriptor.name = "one shot 2";
  job_descriptor.action = [&] { triggered = true; };
  std::optional<JobHandle> handle = scheduler.createJob(job_descriptor);
  if (handle) {
    scheduler.startJob(*handle);
    scheduler.startWaitStop(0.25);
  }
  EXPECT_FALSE(triggered);
}

TEST(Scheduler, Groups) {
  std::vector<ExecutionGroupDescriptor> execution_groups;
  auto clock = std::make_unique<SchedulerClock>();
  Scheduler scheduler(execution_groups, clock.get());
  std::atomic<int> checker(0);
  std::atomic<int> counter(0);
  std::optional<JobHandle> handles[20];
  for (int i = 0; i < 20; i++) {
    JobDescriptor job_descriptor;
    job_descriptor.execution_group = "";
    job_descriptor.target_start_time = NowCount() + SecondsToNano(0.50);
    job_descriptor.priority = 0;
    job_descriptor.slack = 0;
    job_descriptor.execution_mode = ExecutionMode::kOneShotTask;
    job_descriptor.name = "one shot 2";
    job_descriptor.groups.push_back("dummy");
    job_descriptor.action = [&] {
      counter.fetch_add(-1);
      EXPECT_EQ(checker.fetch_add(1), 0);
      Sleep(SecondsToNano(0.05));
      EXPECT_EQ(checker.fetch_add(-1), 1);
    };
    handles[i] = scheduler.createJob(job_descriptor);
    if (handles[i]) {
      scheduler.startJob(*handles[i]);
      counter++;
    }
  }
  scheduler.startWaitStop(1.5);
  EXPECT_EQ(counter, 0);
}

}  // namespace scheduler
}  // namespace isaac
