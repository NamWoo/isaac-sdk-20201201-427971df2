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

TEST(Scheduler, PeriodAccurcay) {
  int count = 0;
  int64_t last = -1;
  auto clock = std::make_unique<SchedulerClock>();
  std::vector<ExecutionGroupDescriptor> execution_groups;
  Scheduler scheduler(execution_groups, clock.get());
  JobDescriptor job_descriptor;
  job_descriptor.execution_group = "";
  job_descriptor.period = SecondsToNano(0.10);
  job_descriptor.priority = 0;
  job_descriptor.slack = 0;
  job_descriptor.execution_mode = ExecutionMode::kPeriodicTask;
  job_descriptor.name = "period accuracy";
  job_descriptor.action = [&] {
    const int now = NowCount();
    if (last != -1) {
      EXPECT_NEAR(ToSeconds(now - last), 0.10, 0.002);
    }
    last = now;
    Sleep(SecondsToNano(0.04));
    count++;
  };
  std::optional<JobHandle> handle = scheduler.createJob(job_descriptor);
  if (handle) {
    scheduler.startJob(*handle);
    scheduler.startWaitStop(0.95);
  }
  EXPECT_EQ(count, 10);
}

}  // namespace scheduler
}  // namespace isaac
