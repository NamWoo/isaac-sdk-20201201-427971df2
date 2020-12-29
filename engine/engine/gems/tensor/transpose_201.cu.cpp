/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "transpose_201.cu.hpp"

#include <cuda_runtime.h>

namespace isaac {

namespace {

// CUDA kernel for Transpose201.
// TODO(dweikersdorf) This kernel is far from optimal
__global__ void Transpose201Impl(StridePointer<const float> input, StridePointer<float> output,
                                 int rows, int cols, int channels) {
  const int row = blockIdx.y;
  int col = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows || col >= cols) return;

  const float* data = input.row_pointer(row) + channels * col;
  int output_offset = row * cols + col;
  while (col < cols) {
    for (int i = 0; i < channels; i++) {
      *(output.row_pointer(i) + output_offset) = data[i];
    }
    const int step = blockDim.x * gridDim.x;
    col += step;
    data += channels * step;
    output_offset += step;
  }
}

}  // namespace

void Transpose201(StridePointer<const float> input, StridePointer<float> output,
                  int rows, int cols, int channels) {
  dim3 block(256, 1, 1);
  dim3 grid(1, rows, 1);
  Transpose201Impl<<<grid, block>>>(input, output, rows, cols, channels);
}

}  // namespace isaac
