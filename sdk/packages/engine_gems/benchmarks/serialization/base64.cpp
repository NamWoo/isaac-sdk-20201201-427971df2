/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <random>
#include <string>

#include "benchmark/benchmark.h"
#include "engine/core/image/image.hpp"
#include "engine/gems/serialization/base64.hpp"

isaac::Image3ub DummyImage(int rows, int cols) {
  isaac::Image3ub image(rows, cols);
  std::default_random_engine rng;
  std::uniform_int_distribution<uint8_t> rnd(0, 255);
  for (int i = 0; i < image.num_pixels(); i++) {
    image[i] = isaac::Pixel3ub{rnd(rng), rnd(rng), rnd(rng)};
  }
  return image;
}

void Base64_Image(benchmark::State& state) {
  isaac::Image3ub image = DummyImage(640, 480);
  size_t count = 0;
  for (auto _ : state) {
    const std::string str = isaac::serialization::Base64Encode(image);
    count += str.size();
  }
}

void Base64_Buffer(benchmark::State& state) {
  isaac::Image3ub image = DummyImage(640, 480);
  size_t count = 0;
  for (auto _ : state) {
    const std::string str = isaac::serialization::Base64Encode(image.element_wise_begin(),
                                                               image.num_elements());
    count += str.size();
  }
}

BENCHMARK(Base64_Image);
BENCHMARK(Base64_Buffer);
BENCHMARK_MAIN();
