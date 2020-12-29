/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <atomic>
#include <memory>
#include <queue>
#include <shared_mutex>  // NOLINT
#include <unordered_map>

#include "engine/core/allocator/allocator_base.hpp"

namespace isaac {

// Allocator which uses a cache in combination with another allocator to optimize repeated
// allocations and deallocations of the same size.
class CachedAllocator : public AllocatorBase {
 public:
  CachedAllocator(std::unique_ptr<AllocatorBase> child_allocator);
  ~CachedAllocator();

  pointer_t allocateBytes(size_t size) override;
  void deallocateBytes(pointer_t pointer_t, size_t size) override;

  // Switches to cached mode with given number of buckets. Buckets are chosen based on collected
  // statistics. Can only called once during the lifetime of an object. This will invalidate
  // statistics collected so far.
  void switchToCachedMode(size_t num_buckets);

  // Deallocates all memory. Should only be called just before the end of the lifetime of an
  // object.
  void finalize();

  // Gets the total number of times memory was allocated.
  size_t getRequestCount() const;
  // Gets the total number of bytes which have been requested.
  size_t getTotalBytesRequested() const;
  // Gets the total number of bytes which have actually been allocated by the underlying allocator.
  size_t getTotalBytesAllocated() const;
  // Gets the total number of bytes which have been deallocated by the underlying allocator.
  size_t getTotalBytesDeallocated() const;

 private:
  // The mode in which the allocator is currently in
  enum class Mode {
    // All allocation and deallocation requests are directly forwarded to the underlying allocator.
    // The allocator starts in this mode.
    Direct,
    // All allocation requests are first checked against an internal storage of buffers.
    // Deallocation requests add buffers to the internal storage. The allocator can trafers from
    // `Direct` mode to `Cached` mode exactly once during its lifetime.
    Cached
  };

  // A helper type holding the implementation details of this class.
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace isaac
