/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/tensor/transpose.hpp"

#include "gtest/gtest.h"

namespace isaac {

TEST(Tensor, TransposeCpu) {
  Tensor3i source(6, 11, 5);
  for (int i = 0; i < source.dimensions()[0]; i++) {
    for (int j = 0; j < source.dimensions()[1]; j++) {
      for (int k = 0; k < source.dimensions()[2]; k++) {
        source(i, j, k) = (i + 3) * (j + 5) * (k + 7);
      }
    }
  }

  Tensor3i result;
  Transpose(source.const_view(), TensorTransposeOp::k201, result);

  ASSERT_EQ(result.dimensions()[0], source.dimensions()[2]);
  ASSERT_EQ(result.dimensions()[1], source.dimensions()[0]);
  ASSERT_EQ(result.dimensions()[2], source.dimensions()[1]);

  for (int i = 0; i < result.dimensions()[0]; i++) {
    for (int j = 0; j < result.dimensions()[1]; j++) {
      for (int k = 0; k < result.dimensions()[2]; k++) {
        ASSERT_EQ(result(i, j, k), source(j, k, i));
      }
    }
  }
}

TEST(Tensor, TransposeCuda) {
  std::vector<Vector3i> test_dimensions = {
    Vector3i(6, 11, 5),
    Vector3i(600, 1100, 3)
  };

  for (const auto& dimensions : test_dimensions) {
    Tensor3f source(dimensions);
    for (int i = 0; i < source.dimensions()[0]; i++) {
      for (int j = 0; j < source.dimensions()[1]; j++) {
        for (int k = 0; k < source.dimensions()[2]; k++) {
          source(i, j, k) = (i + 3) * (j + 5) * (k + 7);
        }
      }
    }

    CudaTensor3f source_cuda(source.dimensions());
    Copy(source, source_cuda);

    CudaTensor3f result_cuda;
    Transpose(source_cuda.const_view(), TensorTransposeOp::k201, result_cuda);

    Tensor3f result(result_cuda.dimensions());
    Copy(result_cuda, result);

    ASSERT_EQ(result.dimensions()[0], source.dimensions()[2]);
    ASSERT_EQ(result.dimensions()[1], source.dimensions()[0]);
    ASSERT_EQ(result.dimensions()[2], source.dimensions()[1]);

    for (int i = 0; i < result.dimensions()[0]; i++) {
      for (int j = 0; j < result.dimensions()[1]; j++) {
        for (int k = 0; k < result.dimensions()[2]; k++) {
          ASSERT_EQ(result(i, j, k), source(j, k, i));
        }
      }
    }
  }
}

}  // namespace isaac
