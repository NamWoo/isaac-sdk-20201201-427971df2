/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "allocator_backend.hpp"

#include <string>
#include <thread>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/core/allocator/allocators.hpp"
#include "engine/core/allocator/cached_allocator.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/scheduler/scheduler.hpp"

namespace isaac {
namespace alice {

namespace {

// Time period for which the allocator will collect allocation statistics.
constexpr double kStatisticsCollectionDuration = 10.0;

// Number of buckets to use for cached allocator
constexpr int kNumBuckets = 128;

}  // namespace

AllocatorBackend::AllocatorBackend(Application* app)
: app_(app) { }

void AllocatorBackend::start() {
  start_time_ = -1;

  // Adds a job which will switch the global allocator to cached mode after collecting allocation
  // statistics for a certain time period.
  scheduler::JobDescriptor job_descriptor;
  job_descriptor.priority = 0;
  job_descriptor.execution_mode = scheduler::ExecutionMode::kOneShotTask;
  job_descriptor.target_start_time = SecondsToNano(kStatisticsCollectionDuration);
  job_descriptor.name = "AllocatorBackend";
  job_descriptor.action = [this] {
    CachedAllocator* cpu_allocator = getCpuAllocator();
    if (cpu_allocator == nullptr) {
      LOG_ERROR("Could not switch to cached allocation mode as the global CPU allocator is not of "
                "type CachedAllocator");
    } else {
      cpu_allocator->switchToCachedMode(kNumBuckets);
      LOG_INFO("Optimized memory CPU allocator.");
    }

    CachedAllocator* cuda_allocator = getCudaAllocator();
    if (cuda_allocator == nullptr) {
      LOG_ERROR("Could not switch to cached allocation mode as the global CUDA allocator is not of "
                "type CachedAllocator");
    } else {
      cuda_allocator->switchToCachedMode(kNumBuckets);
      LOG_INFO("Optimized memory CUDA allocator.");
    }

    start_time_ = NowCount();
  };
  app_->backend()->scheduler()->createJobAndStart(job_descriptor);
}

void AllocatorBackend::stop() {
  if (CachedAllocator* allocator = getCpuAllocator()) {
    allocator->finalize();
  }
  if (CachedAllocator* allocator = getCudaAllocator()) {
    allocator->finalize();
  }
}

double AllocatorBackend::duration() const {
  if (start_time_ == -1) {
    return 0.0;
  } else {
    return ToSeconds(NowCount() - start_time_);
  }
}

CachedAllocator* AllocatorBackend::getCpuAllocator() const {
  return dynamic_cast<CachedAllocator*>(GetCpuAllocator());
}

CachedAllocator* AllocatorBackend::getCudaAllocator() const {
  return dynamic_cast<CachedAllocator*>(GetCudaAllocator());
}

}  // namespace alice
}  // namespace isaac
