/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "yuv2rgb.cu.hpp"

namespace {

// Helper function to ensure valid image data ranges
__device__ inline unsigned char ClampToUByte(float value) {
  const float maximum = 255.0f;
  const float minimum = 0.0f;
  return (unsigned char)((value >= minimum) ? ((value <= maximum) ? value : maximum) : minimum);
}

// Converts YUV420 encoded as NV21 storage to RGB output images.
// Assumes images are bound to textures for processing.
__global__ void ConvertNv21ToRgbImpl(cudaTextureObject_t y_channel, cudaTextureObject_t uv_channel,
                                      unsigned char* rgb_output, unsigned int width,
                                      unsigned int height, unsigned int output_pitch) {
  int2 y_coords;
  y_coords.x = blockIdx.x * blockDim.x + threadIdx.x;
  y_coords.y = blockIdx.y * blockDim.y + threadIdx.y;

  // Index exceeds image dimension
  if (y_coords.x >= width || y_coords.y >= height) {
    return;
  }
  // uv index varies at half the rate of the y index.
  int2 uv_coords;
  uv_coords.x = y_coords.x / 2;
  uv_coords.y = y_coords.y / 2;

  // Y values are in the x channel
  uchar4 y = tex2D<uchar4>(y_channel, y_coords.x, y_coords.y);
  // u valies in x channel, v values in y channel
  uchar4 uv = tex2D<uchar4>(uv_channel, uv_coords.x, uv_coords.y);

  unsigned int rgb_offset = y_coords.y * output_pitch + 3 * y_coords.x;
  rgb_output[rgb_offset] = ClampToUByte(y.x + (1.370705 * (uv.y - 128)));
  rgb_output[rgb_offset + 1] =
      ClampToUByte(y.x - (0.698001 * (uv.y - 128)) - (0.337633 * (uv.x - 128)));
  rgb_output[rgb_offset + 2] = ClampToUByte(y.x + (1.732446 * (uv.x - 128)));
}
}  // namespace

namespace isaac {

void ConvertNv21ToRgb(cudaTextureObject_t y_channel, cudaTextureObject_t uv_channel,
                      unsigned char* rgb_output, unsigned int width, unsigned int height,
                      unsigned int output_pitch) {
  // Split work into 16 by 16 grids across the images.
  dim3 block(16, 16);
  dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
  ConvertNv21ToRgbImpl<<<grid, block>>>(y_channel, uv_channel, rgb_output, width, height,
                                         output_pitch);
}

}  // namespace isaac
