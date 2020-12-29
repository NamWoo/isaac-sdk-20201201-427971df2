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
#include "engine/core/math/types.hpp"
#include "engine/gems/math/catmull_rom.hpp"

namespace isaac {

// A reference benchmark which executes code which is required for the Sample test, but does not
// actually call the sample function.
void SampleReference(benchmark::State& state) {
  double time = 0.0;
  Vector2d result = Vector2d::Zero();
  for (auto _ : state) {
    Vector2d position = Vector2d::Zero();
    result += position;
    time += 0.01;
    if (time > 4.0) time = 0.0;
  }
  LOG_INFO("%f, %f", result[0], result[1]);
}

// Evaluates a spline at different curve times.
void Sample(benchmark::State& state) {
  const Vector2d points[4] = {
      Vector2d{0.303656, 0.316798}, Vector2d{0.677207, 1.17211},
      Vector2d{0.896317, 1.72843}, Vector2d{1.0771, 1.91572}};
  double times[4];
  times[0] = 0.0;
  CatmullRomComputeCurveTimes(points, times, 4);

  double time = 0.0;
  Vector2d result = Vector2d::Zero();
  for (auto _ : state) {
    Vector2d position;
    CatmullRomSplineEvaluate(points, times, time, &position);
    result += position;
    time += 0.01;
    if (time > times[3]) time = 0.0;
  }
  LOG_INFO("%f, %f", result[0], result[1]);
}

// Evaluates a spline at different curve times and computes both position and tangent.
void SampleWithTangent(benchmark::State& state) {
  const Vector2d points[4] = {
      Vector2d{0.303656, 0.316798}, Vector2d{0.677207, 1.17211},
      Vector2d{0.896317, 1.72843}, Vector2d{1.0771, 1.91572}};
  double times[4];
  times[0] = 0.0;
  CatmullRomComputeCurveTimes(points, times, 4);

  double time = 0.0;
  Vector2d result1 = Vector2d::Zero();
  Vector2d result2 = Vector2d::Zero();
  for (auto _ : state) {
    Vector2d position, tangent;
    CatmullRomSplineEvaluate(points, times, time, &position, &tangent);
    result1 += position;
    result2 += tangent;
    time += 0.01;
    if (time > times[3]) time = 0.0;
  }
  LOG_INFO("%f, %f, %f, %f", result1[0], result1[1], result2[0], result2[1]);
}

BENCHMARK(SampleReference);
BENCHMARK(Sample);
BENCHMARK(SampleWithTangent);

}  // namespace isaac

BENCHMARK_MAIN();
