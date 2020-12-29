/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <random>
#include <vector>

#include "engine/core/allocator/cached_allocator.hpp"
#include "engine/core/allocator/malloc_allocator.hpp"
#include "engine/core/allocator/test_utils.hpp"
#include "engine/core/logger.hpp"
#include "gtest/gtest.h"

namespace isaac {

namespace {

void DirectTestImpl(std::function<size_t()> size_cb) {
  CachedAllocator allocator(std::make_unique<MallocAllocator>());
  BatchAllocDealloc(allocator, 1000, 10, size_cb);
  allocator.finalize();
  EXPECT_EQ(allocator.getTotalBytesAllocated(), allocator.getTotalBytesDeallocated());
  LOG_INFO("Requested:   %zd", allocator.getTotalBytesRequested());
  LOG_INFO("Allocated:   %zd", allocator.getTotalBytesAllocated());
  LOG_INFO("Deallocated: %zd", allocator.getTotalBytesDeallocated());
}

void CachedTestImpl(std::function<size_t()> size_cb) {
  CachedAllocator allocator(std::make_unique<MallocAllocator>());
  BatchAllocDealloc(allocator, 10, 10, size_cb);
  allocator.switchToCachedMode(32);
  BatchAllocDealloc(allocator, 990, 10, size_cb);
  allocator.finalize();
  EXPECT_EQ(allocator.getTotalBytesAllocated(), allocator.getTotalBytesDeallocated());
  LOG_INFO("Requested:   %zd", allocator.getTotalBytesRequested());
  LOG_INFO("Allocated:   %zd", allocator.getTotalBytesAllocated());
  LOG_INFO("Deallocated: %zd", allocator.getTotalBytesDeallocated());
}

}  // namespace

TEST(Allocator, AllocDealloc_100_1) {
  CachedAllocator allocator(std::make_unique<MallocAllocator>());
  float* ptr = allocator.allocate<float>(100);
  allocator.deallocate<float>(ptr, 100);
}

TEST(Allocator, AllocDealloc_100_1M) {
  CachedAllocator allocator(std::make_unique<MallocAllocator>());
  for (int i = 0; i < 1'000'000; i++) {
    float* ptr = allocator.allocate<float>(100);
    allocator.deallocate<float>(ptr, 100);
  }
}

TEST(Allocator, Alloc_100_10K) {
  DirectTestImpl([&] { return 100; });
}

TEST(Allocator, Alloc_Rng_10K) {
  std::default_random_engine rng;
  std::uniform_int_distribution<size_t> random_size(1, 100);
  DirectTestImpl([&] { return random_size(rng) * 377; });
}

TEST(Allocator, Alloc_Some_10K) {
  const std::vector<size_t> sizes{{100, 320*240, 4*320*240}};
  std::default_random_engine rng;
  std::discrete_distribution<> random_size({1, 100, 10});
  DirectTestImpl([&] { return random_size(rng); });
}

TEST(Allocator, AllocDirectGammaPool) {
  DirectTestImpl(RandomSizeGamma(25));
}

TEST(Allocator, AllocCachedGammaPool) {
  CachedTestImpl(RandomSizeGamma(25));
}

TEST(Allocator, AllocDirectGamma) {
  DirectTestImpl(RandomSizeGamma());
}

TEST(Allocator, AllocCachedGamma) {
  CachedTestImpl(RandomSizeGamma());
}

}  // namespace isaac
