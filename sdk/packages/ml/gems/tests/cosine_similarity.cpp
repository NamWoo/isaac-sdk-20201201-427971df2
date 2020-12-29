/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/ml/gems/cosine_similarity.hpp"

#include <random>

#include "cublas_v2.h"
#include "cuda_runtime.h"
#include "engine/core/optional.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace ml {
namespace {
constexpr int kNumRows = 1024;
constexpr int kNumCols = 256;
}  // namespace

TEST(CosineSimilarity, initialize) {
  Tensor2f data(kNumRows, kNumCols);
  CosineSimilarity similarity(data);
  EXPECT_EQ(similarity.getSampleCount(), kNumRows);
  EXPECT_EQ(similarity.getChannelCount(), kNumCols);
}

TEST(CosineSimilarity, initialize_normalize) {
  Tensor2f data(kNumRows, kNumCols);
  CosineSimilarity similarity(data, true);
  EXPECT_EQ(similarity.getSampleCount(), kNumRows);
  EXPECT_EQ(similarity.getChannelCount(), kNumCols);
}

TEST(CosineSimilarity, nearestIndexPerfect) {
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  Tensor2f data(kNumRows, kNumCols);
  for (int row = 0; row < kNumRows; row++) {
    for (int col = 0; col < kNumCols; col++) {
      data(row, col) = distribution(generator);
    }
  }
  Tensor1f vector(kNumCols);
  CudaTensor1f cuda_data(kNumCols);

  cublasContext* handle;
  const cublasStatus_t cublas_result = cublasCreate(&handle);
  ASSERT_EQ(cublas_result, CUBLAS_STATUS_SUCCESS);
  // Failed before initialization
  CosineSimilarity similarity(data);

  for (int row = 0; row < kNumRows; row++) {
    std::copy(&data(row, 0), &data(row, 0) + kNumCols, &vector(0));
    Copy(vector, cuda_data);
    EXPECT_EQ(similarity.nearestIndex(cuda_data.const_view(), handle), row);
  }

  cublasDestroy(handle);
}

TEST(CosineSimilarity, nearestIndexApproximateScaleWithoutNormalize) {
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-1.0,1.0);
  Tensor2f data(kNumRows, kNumCols);
  for (int row = 0; row < kNumRows; row++) {
    for (int col = 0; col < kNumCols; col++) {
      data(row, col) = distribution(generator) * (1+row);
    }
  }

  cublasContext* handle;
  const cublasStatus_t cublas_result = cublasCreate(&handle);
  ASSERT_EQ(cublas_result, CUBLAS_STATUS_SUCCESS);
  CosineSimilarity similarity(data, false);

  Tensor1f vector(kNumCols);
  CudaTensor1f cuda_data(kNumCols);

  int count = 0;
  for (int row = 0; row < kNumRows; row++) {
    std::copy(&data(row, 0), &data(row, 0) + kNumCols, &vector(0));
    Copy(vector, cuda_data);
    count += similarity.nearestIndex(cuda_data, handle) == row;
  }
  EXPECT_LT(count, kNumRows);

  cublasDestroy(handle);
}

TEST(CosineSimilarity, nearestIndexApproximateScale) {
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-1.0,1.0);
  Tensor2f data(kNumRows, kNumCols);
  for (int row = 0; row < kNumRows; row++) {
    for (int col = 0; col < kNumCols; col++) {
      data(row, col) = distribution(generator) * (1+row);
    }
  }

  cublasContext* handle;
  const cublasStatus_t cublas_result = cublasCreate(&handle);
  ASSERT_EQ(cublas_result, CUBLAS_STATUS_SUCCESS);
  CosineSimilarity similarity(data, true);

  Tensor1f vector(kNumCols);
  CudaTensor1f cuda_data(kNumCols);

  for (int row = 0; row < kNumRows; row++) {
    std::copy(&data(row, 0), &data(row, 0) + kNumCols, &vector(0));
    Copy(vector, cuda_data);
    EXPECT_EQ(similarity.nearestIndex(cuda_data, handle), row);
  }

  cublasDestroy(handle);
}

TEST(CosineSimilarity, nearestIndexApproximateRandom) {
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-1.0,1.0);
  Tensor2f data(kNumRows, kNumCols);
  for (int row = 0; row < kNumRows; row++) {
    for (int col = 0; col < kNumCols; col++) {
      data(row, col) = distribution(generator);
    }
  }

  cublasContext* handle;
  const cublasStatus_t cublas_result = cublasCreate(&handle);
  ASSERT_EQ(cublas_result, CUBLAS_STATUS_SUCCESS);
  CosineSimilarity similarity(data, true);

  Tensor1f vector(kNumCols);
  CudaTensor1f cuda_data(kNumCols);

  for (int row = 0; row < kNumRows; row++) {
    std::copy(&data(row, 0), &data(row, 0) + kNumCols, &vector(0));
    Copy(vector, cuda_data);
    EXPECT_EQ(similarity.nearestIndex(cuda_data, handle), row);
  }

  cublasDestroy(handle);
}
}  // namespace ml
}  // namespace isaac
