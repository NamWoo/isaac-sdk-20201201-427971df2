/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <random>

#include "benchmark/benchmark.h"
#include "engine/core/math/benchmarks/cache.hpp"
#include "engine/core/math/utils.hpp"

template <typename K>
void Floor(benchmark::State& state) {
  std::default_random_engine rng;
  std::uniform_real_distribution<K> rnd(-1000.0, 1000.0);
  Cache256<K> c([&] { return rnd(rng); });
  for (auto _ : state) {
    for (int i = 0; i < 1024; i++) {
      benchmark::DoNotOptimize(isaac::FloorToInt(c()));
    }
  }
}

template <typename K>
int FastFloor(K x) {
  const int i = static_cast<int>(x);
  if (i < K(0) && static_cast<K>(i) != x) {
    return i - 1;
  } else {
    return i;
  }
}

template <typename K>
void FastFloor(benchmark::State& state) {
  std::default_random_engine rng;
  std::uniform_real_distribution<K> rnd(-1000.0, 1000.0);
  Cache256<K> c([&] { return rnd(rng); });
  for (auto _ : state) {
    for (int i = 0; i < 1024; i++) {
      benchmark::DoNotOptimize(FastFloor(c()));
    }
  }
}

void FloorF32(benchmark::State& state) { Floor<float>(state); }
void FloorF64(benchmark::State& state) { Floor<double>(state); }
void FastFloorF32(benchmark::State& state) { FastFloor<float>(state); }
void FastFloorF64(benchmark::State& state) { FastFloor<double>(state); }

BENCHMARK(FloorF32);
BENCHMARK(FloorF64);
BENCHMARK(FastFloorF32);
BENCHMARK(FastFloorF64);
BENCHMARK_MAIN();
