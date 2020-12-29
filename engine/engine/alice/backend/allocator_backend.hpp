/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <string>

#include "engine/core/allocator/cached_allocator.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace alice {

class Application;

// Manages memory allocator
class AllocatorBackend  {
 public:
  AllocatorBackend(Application* app);

  void start();
  void stop();

  // Total duration (in seconds) for which cached allocators have been running
  double duration() const;

  // Gets the CPU cached allocator (or null if cached allocator is not yet used)
  CachedAllocator* getCpuAllocator() const;

  // Gets the GPU cached allocator (or null if cached allocator is not yet used)
  CachedAllocator* getCudaAllocator() const;

 private:
  Application* app_;
  int64_t start_time_;
};

}  // namespace alice
}  // namespace isaac
