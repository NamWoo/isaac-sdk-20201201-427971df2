/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <utility>

#include "engine/core/tensor/tensor.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {

// Indicates which tensor transpose operation is executed.
enum class TensorTransposeOp {
  k012,  // identity
  k201   // out(i,j,k) = in(j,k,i)
};

// Use strings when serializing ImageToTensorIndexOrder enum
NLOHMANN_JSON_SERIALIZE_ENUM(TensorTransposeOp, {
    {TensorTransposeOp::k012, "012"},
    {TensorTransposeOp::k201, "201"},
});

// Transposes a rank-3 tensor for example from (A, B, C) to (C, A, B). The transpose operation
// is defined by the argument `op`. See corresponding comments for more information.
template <typename K>
void Transpose(TensorBase<K, tensor::Rank<3>, CpuBufferConstView> input, TensorTransposeOp op,
               TensorBase<K, tensor::Rank<3>, CpuBufferView> output) {
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
      for (int channel = 0; channel < channels; channel++) {
        const K* src = input.element_wise_begin() + channel;
        K* dst = output.slice(channel).element_wise_begin();
        const int count = rows * cols;
        const K* dst_end = dst + count;
        for (; dst != dst_end; ++dst, src+=channels) {
          *dst = *src;
        }
      }
    } break;
    default: PANIC("Unsupported operation");
  }
}
template <typename K>
void Transpose(TensorBase<K, tensor::Rank<3>, CpuBufferConstView> input, TensorTransposeOp op,
               TensorBase<K, tensor::Rank<3>, CpuBuffer>& output) {
  const int rows = input.dimensions()[0];
  const int cols = input.dimensions()[1];
  const int channels = input.dimensions()[2];

  switch (op) {
    case TensorTransposeOp::k012: {
      output.resize(rows, cols, channels);
      Transpose(input, op, output.view());
    } break;
    case TensorTransposeOp::k201: {
      output.resize(channels, rows, cols);
      Transpose(input, op, output.view());
    } break;
    default: PANIC("Unsupported operation");
  }
}

// Transposes a rank-3 tensor for example from (A, B, C) to (B, C, A). Currently only a limited
// number of operations are supported
void Transpose(CudaTensorConstView3f input, TensorTransposeOp op, CudaTensor3f& output);
void Transpose(CudaTensorConstView3f input, TensorTransposeOp op, CudaTensorView3f output);

}  // namespace isaac
