/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/time.hpp"
#include "engine/gems/scheduler/clock.hpp"
#include "engine/gems/scheduler/scheduler.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace scheduler {

class SchedulerClock : public scheduler::Clock {
 public:
  int64_t now() override { return NowCount(); }
  void advance(int64_t dt) override {}
};

TEST(Scheduler, WaitForFinish) {
  std::vector<ExecutionGroupDescriptor> execution_groups;
  auto clock = std::make_unique<SchedulerClock>();
  Scheduler scheduler(execution_groups, clock.get());
  std::vector<JobHandle> jobs;
  std::set<JobHandle> done;
  for (int i = 0; i < 100; i++) {
    JobDescriptor job_descriptor;
    job_descriptor.execution_group = "";
    job_descriptor.target_start_time = NowCount() + SecondsToNano(0.50);
    job_descriptor.priority = 0;
    job_descriptor.slack = 0;
    job_descriptor.execution_mode = ExecutionMode::kBlocking;
    job_descriptor.name = "wait for finish job";
    job_descriptor.action = [i, &jobs, &done] {
      EXPECT_EQ(done.count(jobs[i]), 0);
      Sleep(SecondsToNano(0.001));
    };
    std::optional<JobHandle> handle = scheduler.createJob(job_descriptor);
    if (handle) {
      scheduler.startJob(*handle);
      jobs.push_back(*handle);
    }
  }
  scheduler.startWaitStop(1.00);
  for (const JobHandle& handle : jobs) {
    scheduler.destroyJob(handle);
    scheduler.waitForJobDestruction(handle);
    done.insert(handle);
  }
}

}  // namespace scheduler
}  // namespace isaac
