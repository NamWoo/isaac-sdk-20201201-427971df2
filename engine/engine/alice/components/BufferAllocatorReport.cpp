/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "BufferAllocatorReport.hpp"

#include <string>
#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/allocator_backend.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/redis_backend.hpp"
#include "engine/core/allocator/allocators.hpp"

namespace isaac {
namespace alice {

void BufferAllocatorReport::start() {
  tickPeriodically(3.0);
}

void BufferAllocatorReport::tick() {
  const std::string app_str = node()->app()->uuid().str();

  AllocatorBackend& allocator_backend = *node()->app()->backend()->allocator_backend();

  const double duration = allocator_backend.duration();
  const std::pair<std::string, CachedAllocator*> allocators[2] = {
    {"cpu", allocator_backend.getCpuAllocator()},
    {"cuda", allocator_backend.getCudaAllocator()}
  };

  RedisBackend& redis = *node()->app()->backend()->redis_backend();

  for (const auto& kvp : allocators) {
    CachedAllocator* allocator = kvp.second;
    if (allocator == nullptr || duration <= 0.0) {
      continue;
    }

    const size_t request_count = allocator->getRequestCount();
    const size_t bytes_requested = allocator->getTotalBytesRequested();
    const size_t bytes_allocated = allocator->getTotalBytesAllocated();

    const double efficiency =
        1.0 - static_cast<double>(bytes_allocated)
              / static_cast<double>(bytes_requested == 0 ? 1 : bytes_requested);

    redis.command("HSET isaac:%s:mem:%s duration %f", app_str.c_str(), kvp.first.c_str(),
                  duration);
    redis.command("HSET isaac:%s:mem:%s count %lld", app_str.c_str(), kvp.first.c_str(),
                  request_count);
    redis.command("HSET isaac:%s:mem:%s rate %f", app_str.c_str(), kvp.first.c_str(),
                  static_cast<double>(request_count) / duration);
    redis.command("HSET isaac:%s:mem:%s bytes_requested %lld", app_str.c_str(), kvp.first.c_str(),
                  bytes_requested);
    redis.command("HSET isaac:%s:mem:%s bytes_allocated %lld", app_str.c_str(), kvp.first.c_str(),
                  bytes_allocated);
    redis.command("HSET isaac:%s:mem:%s bytes_deallocator %lld", app_str.c_str(), kvp.first.c_str(),
                  allocator->getTotalBytesDeallocated());
    redis.command("HSET isaac:%s:mem:%s efficiency %f", app_str.c_str(), kvp.first.c_str(),
                  efficiency);
  }
}

}  // namespace alice
}  // namespace isaac
