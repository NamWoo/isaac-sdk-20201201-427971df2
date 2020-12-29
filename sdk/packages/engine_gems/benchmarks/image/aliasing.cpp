/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <memory>

#include "benchmark/benchmark.h"

struct A {
  size_t n;
  size_t* p;
};

void Foo1(A a);
void Foo2(const A& a);

void Test1(benchmark::State& state) {
  constexpr size_t n = 10'000'000;
  size_t* v = new size_t[n];
  for (auto _ : state) {
    Foo1(A{n, v});
  }
  delete[] v;
}

void Test2(benchmark::State& state) {
  constexpr size_t n = 10'000'000;
  size_t* v = new size_t[n];
  for (auto _ : state) {
    Foo2(A{n, v});
  }
  delete[] v;
}

void Foo1(A a) {
  for (size_t i = 0; i < a.n; i++) {
    a.p[i] = i;
  }
}

void Foo2(const A& a) {
  for (size_t i = 0; i < a.n; i++) {
    a.p[i] = i;
  }
}

// We run tests in both orders to make sure it doesn't depend on the order. We might expect to see
// a difference in execution time, but with our current optimization settings this is not the case.
BENCHMARK(Test1);
BENCHMARK(Test2);
BENCHMARK(Test2);
BENCHMARK(Test1);

BENCHMARK_MAIN();
