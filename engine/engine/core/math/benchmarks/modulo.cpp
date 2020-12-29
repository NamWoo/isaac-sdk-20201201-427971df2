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

void PositiveModule(benchmark::State& state) {
  std::default_random_engine rng;
  std::uniform_int_distribution<int> rnd(-1000, 1000);
  std::uniform_int_distribution<int> pos(1, 1000);
  Cache256<int> c1([&] { return rnd(rng); });
  Cache256<int> c2([&] { return pos(rng); });
  for (auto _ : state) {
    for (int i = 0; i < 1024; i++) {
      benchmark::DoNotOptimize(isaac::PositiveModulo(c1(), c2()));
    }
  }
}

int PositiveModuleAlternativeImpl(int x, int n) {
  return (x % n + n) % n;
}

void PositiveModuleAlternative(benchmark::State& state) {
  std::default_random_engine rng;
  std::uniform_int_distribution<int> rnd(-1000, 1000);
  std::uniform_int_distribution<int> pos(1, 1000);
  Cache256<int> c1([&] { return rnd(rng); });
  Cache256<int> c2([&] { return pos(rng); });
  for (auto _ : state) {
    for (int i = 0; i < 1024; i++) {
      benchmark::DoNotOptimize(PositiveModuleAlternativeImpl(c1(), c2()));
    }
  }
}

BENCHMARK(PositiveModule);
BENCHMARK(PositiveModuleAlternative);
BENCHMARK_MAIN();
