/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <memory>
#include <sstream>

#include "benchmark/benchmark.h"
#include "engine/gems/sight/sop.hpp"

void Sight_CanvasOp_Circles(benchmark::State& state) {
  for (auto _ : state) {
    isaac::sight::Sop sop;
    const int n = state.range(0);
    for (int i = 0; i < n; i++) {
      sop.add(isaac::geometry::CircleF{{0.0f, 0.0f}, 1.0f}, "#000");
    }
  }
}

void Sight_CanvasOp_Circles_WithStringify(benchmark::State& state) {
  for (auto _ : state) {
    isaac::sight::Sop sop;
    const int n = state.range(0);
    for (int i = 0; i < n; i++) {
      sop.add(isaac::geometry::CircleF{{0.0f, 0.0f}, 1.0f}, "#000");
    }
    auto json = sop.moveJson();
    json.dump();
  }
}

BENCHMARK(Sight_CanvasOp_Circles)->Range(10, 10000);
BENCHMARK(Sight_CanvasOp_Circles_WithStringify)->Range(10, 10000);
BENCHMARK_MAIN();
