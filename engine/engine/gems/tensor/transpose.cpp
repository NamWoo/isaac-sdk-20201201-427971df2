/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/tensor/transpose.hpp"

#include "engine/gems/tensor/transpose_201.cu.hpp"
#include "engine/gems/tensor/utils.hpp"

namespace isaac {

void Transpose(CudaTensorConstView3f input, TensorTransposeOp op, CudaTensor3f& output) {
  const int rows = input.dimensions()[0];
  const int cols = input.dimensions()[1];
  const int channels = input.dimensions()[2];

  switch (op) {
    case TensorTransposeOp::k012: {
      output.resize(rows, cols, channels);
      Copy(input, output);
    } break;
    case TensorTransposeOp::k201: {
      output.resize(channels, rows, cols);
      Transpose201({input.element_wise_begin(), cols * channels * sizeof(float)},
                   {output.element_wise_begin(), rows * cols * sizeof(float)},
                   rows, cols, channels);
    } break;
    default: PANIC("Unsupported operation");
  }
}

void Transpose(CudaTensorConstView3f input, TensorTransposeOp op, CudaTensorView3f output) {
  const int rows = input.dimensions()[0];
  const int cols = input.dimensions()[1];
  const int channels = input.dimensions()[2];

  switch (op) {
    case TensorTransposeOp::k012: {
      ISAAC_ASSERT_EQ(rows, output.dimensions()[0]);
      ISAAC_ASSERT_EQ(cols, output.dimensions()[1]);
      ISAAC_ASSERT_EQ(channels, output.dimensions()[2]);
      Copy(input, output);
    } break;
    case TensorTransposeOp::k201: {
      ISAAC_ASSERT_EQ(rows, output.dimensions()[1]);
      ISAAC_ASSERT_EQ(cols, output.dimensions()[2]);
      ISAAC_ASSERT_EQ(channels, output.dimensions()[0]);
      Transpose201({input.element_wise_begin(), cols * channels * sizeof(float)},
                   {output.element_wise_begin(), rows * cols * sizeof(float)},
                   rows, cols, channels);
    } break;
    default: PANIC("Unsupported operation");
  }
}

}  // namespace isaac
