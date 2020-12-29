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
#include <vector>

#include "benchmark/benchmark.h"
#include "engine/gems/image/io.hpp"

namespace isaac {

isaac::Image3ub RandomImage(int rows, int cols) {
  isaac::Image3ub img(rows, cols);
  uint32_t seed;
  // TODO(dweikersdorf) Use image create function
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    img[pixel][0] = rand_r(&seed);
    img[pixel][1] = rand_r(&seed);
    img[pixel][2] = rand_r(&seed);
  }
  return img;
}
isaac::Image1ub RandomDepthImage(int rows, int cols) {
  isaac::Image1ub img(rows, cols);
  uint32_t seed;
  // TODO(dweikersdorf) Use image create function
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    img[pixel] = rand_r(&seed);
  }
  return img;
}
isaac::Image3ub RealImage() {
  isaac::Image3ub img;
  LoadPng("engine/gems/image/data/left.png", img);
  return img;
}
isaac::Image1ub RealDepthLImage() {
  isaac::Image1ub img;
  LoadPng("engine/gems/image/data/depth.png", img);
  return img;
}

void EncodeRandomDepthPNG(benchmark::State& state) {
  int length = 0;
  const int n = state.range(0);
  const auto img = RandomDepthImage(n, n);
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodePng(img, out);
    length += out.size();
  }
}

void EncodeRandomDepthJPG(benchmark::State& state) {
  int length = 0;
  const int n = state.range(0);
  const auto img = RandomDepthImage(n, n);
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodeJpeg(img, 75, out);
    length += out.size();
  }
}

void EncodeRandomPNG(benchmark::State& state) {
  int length = 0;
  const int n = state.range(0);
  const auto img = RandomImage(n, n);
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodePng(img, out);
    length += out.size();
  }
}

void EncodeRandomJPG(benchmark::State& state) {
  int length = 0;
  const int n = state.range(0);
  const auto img = RandomImage(n, n);
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodeJpeg(img, 75, out);
    length += out.size();
  }
}

void EncodeRealPNG(benchmark::State& state) {
  int length = 0;
  const auto img = RealImage();
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodePng(img, out);
    length += out.size();
  }
}

void EncodeRealJPG(benchmark::State& state) {
  int length = 0;
  const auto img = RealImage();
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodeJpeg(img, 75, out);
    length += out.size();
  }
}

void EncodeRealDepthPNG(benchmark::State& state) {
  int length = 0;
  const auto img = RealDepthLImage();
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodePng(img, out);
    length += out.size();
  }
}

void EncodeRealDepthJPG(benchmark::State& state) {
  int length = 0;
  const auto img = RealDepthLImage();
  for (auto _ : state) {
    std::vector<uint8_t> out;
    EncodeJpeg(img, 75, out);
    length += out.size();
  }
}

BENCHMARK(EncodeRandomPNG)->Range(16, 1024)->Unit(benchmark::kMillisecond);;
BENCHMARK(EncodeRandomJPG)->Range(16, 1024)->Unit(benchmark::kMillisecond);;
BENCHMARK(EncodeRealPNG)->Unit(benchmark::kMillisecond);;
BENCHMARK(EncodeRealJPG)->Unit(benchmark::kMillisecond);;

BENCHMARK(EncodeRandomDepthPNG)->Range(16, 1024)->Unit(benchmark::kMillisecond);;
BENCHMARK(EncodeRandomDepthJPG)->Range(16, 1024)->Unit(benchmark::kMillisecond);;
BENCHMARK(EncodeRealDepthPNG)->Unit(benchmark::kMillisecond);;
BENCHMARK(EncodeRealDepthJPG)->Unit(benchmark::kMillisecond);;
}  // namespace isaac

BENCHMARK_MAIN();
