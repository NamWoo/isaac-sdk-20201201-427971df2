/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "benchmark/benchmark.h"
#include "cuda_runtime.h"
#include "engine/core/image/image.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"

namespace isaac {

void NormalizeRgbCpu(benchmark::State& state) {
  Image3ub image;
  LoadPng("engine/gems/image/data/left.png", image);

  Tensor3f result(3, image.rows(), image.cols());

  for (auto _ : state) {
    ImageToNormalizedTensor(image, result, ImageToTensorNormalization::kUnit);
  }
}

void NormalizeRgbGpu(benchmark::State& state) {
  Image3ub image;
  LoadPng("engine/gems/image/data/left.png", image);
  CudaImage3ub cuda_image(image.dimensions());
  Copy(image, cuda_image);

  CudaTensor3f cuda_result(3, image.rows(), image.cols());

  for (auto _ : state) {
    ImageToNormalizedTensor(cuda_image, cuda_result, ImageToTensorNormalization::kUnit);
    cudaDeviceSynchronize();
  }
}

void NormalizeSingleChannelCpu(benchmark::State& state) {
  Image1ub image;
  LoadPng("engine/gems/image/data/label.png", image);

  Tensor2f result(image.rows(), image.cols());

  for (auto _ : state) {
    ImageToNormalizedTensor(image, result, ImageToTensorNormalization::kUnit);
  }
}

void NormalizeSingleChannelGpu(benchmark::State& state) {
  Image1ub image;
  LoadPng("engine/gems/image/data/label.png", image);
  CudaImage1ub cuda_image(image.dimensions());
  Copy(image, cuda_image);

  CudaTensor2f cuda_result(image.rows(), image.cols());

  for (auto _ : state) {
    ImageToNormalizedTensor(cuda_image, cuda_result, ImageToTensorNormalization::kUnit);
    cudaDeviceSynchronize();
  }
}

BENCHMARK(NormalizeRgbCpu);
BENCHMARK(NormalizeRgbGpu);
BENCHMARK(NormalizeSingleChannelCpu);
BENCHMARK(NormalizeSingleChannelGpu);


// Results as of March 18, 2020 for the left.png which is 1920 x 1080
//
// PC with GeForce RTX 2080Ti and Intel® Core™ i7-6800K CPU @ 3.40GHz × 12 , 64GB Ram
// ----------------------------------------------------
// Benchmark             Time           CPU Iterations
// ----------------------------------------------------
// NormalizeCpu    5672510 ns    5672366 ns        108
// NormalizeGpu      71009 ns      71005 ns       9764
// TODO(snimmagadda): Add benchmark stats for Nano and Xavier

// Results as of March 18, 2020 for the stairs.png which is 960 x 540
//
// PC with GeForce RTX 2080Ti and Intel® Core™ i7-6800K CPU @ 3.40GHz × 12 , 64GB Ram
// ----------------------------------------------------
// Benchmark             Time           CPU Iterations
// ----------------------------------------------------
// NormalizeCpu    1390057 ns    1389981 ns        498
// NormalizeGpu      22265 ns      22265 ns      30391
// TODO(snimmagadda): Add benchmark stats for Nano and Xavier

// Results as of March 18, 2020 for label.png which is 1920 x 1080
//
// PC with GeForce RTX 2080Ti and Intel® Core™ i7-6800K CPU @ 3.40GHz × 12 , 64GB Ram
// -----------------------------------------------------------------
// Benchmark                       Time              CPU Iterations
// -----------------------------------------------------------------
// NormalizeSingleChannelCpu    1949568 ns    1941252 ns        356
// NormalizeSingleChannelGpu      43593 ns      43449 ns      16107
// TODO(snimmagadda): Add benchmark stats for Nano and Xavier

}  // namespace isaac

BENCHMARK_MAIN();
