/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "cached_allocator.hpp"

#include <algorithm>
#include <memory>
#include <shared_mutex>  // NOLINT
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/lockfree/queue.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"

namespace isaac {

namespace {

// The maximum number of allocation instances to collect and store as statistics.
constexpr size_t kMaxStatisticsCount = 1024;

}  // namespace

struct CachedAllocator::Impl {
  // A helper type to store available memory blocks
  struct Store {
    // The size of stored memory blocks
    size_t size;
    // A lock-free queue which can be used to concurrently store and retreive memory blocks.
    boost::lockfree::queue<pointer_t> available;
  };

  // Adds an allocation request to the statistisc
  void collectStatistics(size_t size) {
    statistics.push(size);
    statistics_count_approximation++;
    if (statistics_count_approximation > kMaxStatisticsCount) {
      statistics.pop(size);
      statistics_count_approximation--;
    }
  }

  // Allocates memory using the underlying allocator
  pointer_t allocateDirect(size_t size) {
    total_bytes_allocated += size;
    return child_allocator->allocateBytes(size);
  }

  // Finds an block of memory potentially using the cache
  pointer_t allocateCached(size_t size) {
    auto it = stores.find(size);
    if (it == stores.end()) {
      // use direct mode fallback
      return allocateDirect(size);
    } else {
      // check if memory is available in cache
      pointer_t handle;
      if (!it->second->available.pop(handle)) {
        // nothing available => allocating more
        handle = allocateDirect(size);
      }
      return handle;
    }
  }

  // Dellocates memory using the underlying allocator
  void deallocateDirect(pointer_t handle, size_t size) {
    total_bytes_deallocated += size;
    child_allocator->deallocateBytes(handle, size);
  }

  // Returns a block of memory potentially keeping it in the cache for later use
  void deallocateCached(pointer_t handle, size_t size) {
    const auto it = stores.find(size);
    if (it == stores.end()) {
      // use direct mode fallback
      deallocateDirect(handle, size);
    } else {
      // store memory for reuse
      it->second->available.push(handle);
    }
  }

  // The underlying allocator used to actually allocate or deallocate memory.
  std::unique_ptr<AllocatorBase> child_allocator;

  // The mode in which the allocator currently is in.
  std::atomic<Mode> mode;

  // Collects the size of allocation requests
  boost::lockfree::queue<size_t> statistics;
  // The approximate (i.e. not thread-safe) number of elements stored in `statistics`
  std::atomic<size_t> statistics_count_approximation;

  // The number of requests
  std::atomic<size_t> request_count;
  // The total number of bytes requested
  std::atomic<size_t> total_bytes_requested;
  // The total number of bytes allocated
  std::atomic<size_t> total_bytes_allocated;
  // The total number of bytes deallocated
  std::atomic<size_t> total_bytes_deallocated;

  // Stores available memory blocks by their size
  std::unordered_map<size_t, std::unique_ptr<Store>> stores;
};

CachedAllocator::CachedAllocator(std::unique_ptr<AllocatorBase> child_allocator) {
  impl_ = std::make_unique<Impl>();
  impl_->child_allocator = std::move(child_allocator);
  impl_->mode = Mode::Direct;
  impl_->statistics_count_approximation = 0;
  impl_->request_count = 0;
  impl_->total_bytes_requested = 0;
  impl_->total_bytes_allocated = 0;
  impl_->total_bytes_deallocated = 0;
}

CachedAllocator::~CachedAllocator() {
  finalize();
  // Print a warning if not all memory was deallocated.
  if (impl_->total_bytes_allocated != impl_->total_bytes_deallocated) {
    LOG_WARNING("Potentially lost %zd bytes of memory (allocated total: %zd)",
                impl_->total_bytes_allocated - impl_->total_bytes_deallocated,
                impl_->total_bytes_allocated.load());
  }
}

auto CachedAllocator::allocateBytes(size_t size) -> pointer_t {
  impl_->request_count++;
  impl_->total_bytes_requested += size;
  impl_->collectStatistics(size);
  switch (impl_->mode) {
    default: PANIC("Invalid mode");
    case Mode::Direct: return impl_->allocateDirect(size);
    case Mode::Cached: return impl_->allocateCached(size);
  }
  return nullptr;
}

void CachedAllocator::deallocateBytes(pointer_t handle, size_t size) {
  switch (impl_->mode) {
    default: PANIC("Invalid mode");
    case Mode::Direct: return impl_->deallocateDirect(handle, size);
    case Mode::Cached: return impl_->deallocateCached(handle, size);
  }
}

void CachedAllocator::switchToCachedMode(size_t num_buckets) {
  if (impl_->mode != Mode::Direct) {
    LOG_ERROR("Cached mode already enabled.");
    return;
  }
  // Get approximately all elements from the queue. Don't care if it gets changed while this code
  // is running. Create histogram of how often elements appear.
  const size_t num_samples = impl_->statistics_count_approximation;
  std::unordered_map<size_t, size_t> samples_histogram;
  for (size_t i = 0; i < num_samples; i++) {
    size_t size;
    if (impl_->statistics.pop(size)) {
      samples_histogram[size]++;
    }
  }
  // Sort by occurrence.
  std::vector<std::pair<size_t, size_t>> samples;
  for (const auto& kvp : samples_histogram) {
    samples.push_back(kvp);
  }
  std::sort(samples.begin(), samples.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.second > rhs.second;
      });
  // Pick top N
  if (samples.size() > num_buckets) {
    samples.erase(samples.begin() + num_buckets, samples.end());
  }
  // Create stores
  for (const auto& sample : samples) {
    const auto ok = impl_->stores.emplace(sample.first, std::make_unique<Impl::Store>());
    ASSERT(ok.second, "Could not create new store");
    ok.first->second->size = sample.first;
  }
  // Flip the switch
  impl_->mode = Mode::Cached;
}

void CachedAllocator::finalize() {
  // De-allocate all memory blocks in queues
  for (auto& kvp : impl_->stores) {
    auto* store = kvp.second.get();
    store->available.consume_all([this, size = store->size] (pointer_t handle) {
      impl_->deallocateDirect(handle, size);
    });
  }
  impl_->stores.clear();
}

size_t CachedAllocator::getRequestCount() const {
  return impl_->request_count;
}

size_t CachedAllocator::getTotalBytesRequested() const {
  return impl_->total_bytes_requested;
}

size_t CachedAllocator::getTotalBytesAllocated() const {
  return impl_->total_bytes_allocated;
}

size_t CachedAllocator::getTotalBytesDeallocated() const {
  return impl_->total_bytes_deallocated;
}

}  // namespace isaac
