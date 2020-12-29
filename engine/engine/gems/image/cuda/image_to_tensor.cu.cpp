/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "image_to_tensor.cu.hpp"

#include <cuda_runtime.h>

#include "engine/core/assert.hpp"

namespace isaac {

namespace {

template<int NC>
__global__ void ImageToTensorImpl(StridePointer<const unsigned char> image,
                                     StridePointer<float> result, size_t rows,
                                     size_t cols, float factor, float bias) {
  const int row = blockIdx.y;
  int col = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= rows || col >= cols) return;

  while (col < cols) {
    const unsigned char* data = image.row_pointer(row) + NC * col;
    // Data is laid out in planar blocks by pixel.
    float* result_data = result.row_pointer(row) + NC * col;
    // Normalize and write results
    for (int i = 0; i < NC; i++) {
      result_data[i] = (float)(data[i]) * factor + bias;
    }
    col += blockDim.x * gridDim.x;
  }
}

}  // namespace

void ImageToTensor(StridePointer<const unsigned char> image, StridePointer<float> result,
                      size_t rows, size_t cols, int channels, float factor, float bias) {
  dim3 block(256, 1, 1);
  dim3 grid(1, rows, 1);
  if (channels == 3) {
    ImageToTensorImpl<3><<<grid, block>>>(image, result, rows, cols, factor, bias);
  } else if (channels == 1) {
    ImageToTensorImpl<1><<<grid, block>>>(image, result, rows, cols, factor, bias);
  } else {
    PANIC("Invalid number of channels: %d, must be either 1 or 3", channels);
  }
}

}  // namespace isaac
