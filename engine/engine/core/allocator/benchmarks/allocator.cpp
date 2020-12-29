/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <atomic>
#include <memory>
#include <vector>

#include "benchmark/benchmark.h"
#include "engine/core/allocator/cached_allocator.hpp"
#include "engine/core/allocator/malloc_allocator.hpp"
#include "engine/core/allocator/test_utils.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"

namespace isaac {

namespace {

constexpr int kBatchSize = 50;

std::unique_ptr<CachedAllocator> parallel_allocator;
std::atomic<int> parallel_fence{0};

void ParallelTestImpl(benchmark::State& state, std::function<void(CachedAllocator&)> init_cb,
                     std::function<void(CachedAllocator&, benchmark::State&)> body_cb) {
  if (state.thread_index == 0) {
    logger::SetSeverity(logger::Severity::NONE);
    parallel_allocator = std::make_unique<CachedAllocator>(std::make_unique<MallocAllocator>());
    init_cb(*parallel_allocator);
  } else {
    while (parallel_fence == 0) {}
  }
  parallel_fence++;
  ASSERT(parallel_allocator.get() != nullptr, "parallel_allocator");

  for (auto _ : state) {
    body_cb(*parallel_allocator, state);
  }

  parallel_fence--;
  if (state.thread_index == 0) {
    while (parallel_fence > 0) {}
    logger::SetSeverity(logger::Severity::ALL);
    parallel_allocator.reset();
  }
}

void SingleDirectTestImpl(benchmark::State& state, std::function<size_t()> size_cb) {
  CachedAllocator allocator(std::make_unique<MallocAllocator>());
  for (auto _ : state) {
    BatchAllocDealloc(allocator, 1, kBatchSize, size_cb);
  }
}

void SingleCachedTestImpl(benchmark::State& state, std::function<size_t()> size_cb) {
  CachedAllocator allocator(std::make_unique<MallocAllocator>());
  BatchAllocDealloc(allocator, 50, kBatchSize, size_cb);
  allocator.switchToCachedMode(32);
  for (auto _ : state) {
    BatchAllocDealloc(allocator, 1, kBatchSize, size_cb);
  }
}

}  // namespace

void Malloc(benchmark::State& state) {
  const size_t size = state.range(0);
  std::vector<void*> ptrs;
  ptrs.reserve(kBatchSize);
  for (auto _ : state) {
    ptrs.clear();
    for (int j = 0; j < kBatchSize; j++) {
      ptrs.push_back(std::malloc(size));
    }
    for (const auto& ptr : ptrs) {
      std::free(ptr);
    }
  }
}

void AllocatorDirectDummy(benchmark::State& state) {
  SingleDirectTestImpl(state, [] { return 0; });
}

void AllocatorDirectFixed(benchmark::State& state) {
  const size_t size = state.range(0);
  SingleDirectTestImpl(state, [=] { return size; });
}

void AllocatorCachedFixed(benchmark::State& state) {
  const size_t size = state.range(0);
  SingleCachedTestImpl(state, [=] { return size; });
}

void AllocatorDirectGammaPool(benchmark::State& state) {
  SingleDirectTestImpl(state, RandomSizeGamma(25));
}

void AllocatorCachedGammaPool(benchmark::State& state) {
  SingleCachedTestImpl(state, RandomSizeGamma(25));
}

void AllocatorDirectGamma(benchmark::State& state) {
  SingleDirectTestImpl(state, RandomSizeGamma());
}

void AllocatorCachedGamma(benchmark::State& state) {
  SingleCachedTestImpl(state, RandomSizeGamma());
}

void AllocatorParallelDirect(benchmark::State& state) {
  ParallelTestImpl(state,
    [] (CachedAllocator&) { },
    [] (CachedAllocator& allocator, benchmark::State& state) {
      BatchAllocDealloc(allocator, 1, 10, [&] { return state.range(0); });
    });
}

void AllocatorParallelDirectGamma(benchmark::State& state) {
  auto size_cb = RandomSizeGamma();
  ParallelTestImpl(state,
    [] (CachedAllocator&) { },
    [&] (CachedAllocator& allocator, benchmark::State& state) {
      BatchAllocDealloc(allocator, 1, 10, size_cb);
    });
}

void AllocatorParallelCachedGamma(benchmark::State& state) {
  auto size_cb = RandomSizeGamma();
  ParallelTestImpl(state,
    [&] (CachedAllocator& allocator) {
      // warmup and switch to cached mode
      BatchAllocDealloc(allocator, 100, 10, size_cb);
      allocator.switchToCachedMode(32);
    },
    [&] (CachedAllocator& allocator, benchmark::State& state) {
      BatchAllocDealloc(allocator, 1, 10, size_cb);
    });
}

BENCHMARK(AllocatorDirectDummy);
BENCHMARK(Malloc)->Range(1024, 10*1024*1024);
BENCHMARK(AllocatorDirectFixed)->Range(1024, 10*1024*1024);
BENCHMARK(AllocatorCachedFixed)->Range(1024, 10*1024*1024);
BENCHMARK(AllocatorDirectGammaPool);
BENCHMARK(AllocatorCachedGammaPool);
BENCHMARK(AllocatorDirectGamma);
BENCHMARK(AllocatorCachedGamma);
// BENCHMARK(AllocatorParallelDirect)->ThreadRange(1, 32)->Range(16, 1024*1024);
BENCHMARK(AllocatorParallelDirectGamma)->ThreadRange(1, 32);
BENCHMARK(AllocatorParallelCachedGamma)->ThreadRange(1, 32);

}  // namespace isaac

BENCHMARK_MAIN();
