/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/tensor/universal_tensor.hpp"

#include "gtest/gtest.h"

namespace isaac {

TEST(UniversalTensor, CpuEmpty) {
  CpuUniversalTensorConstView universal_view;
  EXPECT_EQ(universal_view.element_type(), ElementType::kUnknown);
  EXPECT_EQ(universal_view.rank(), 0);
  EXPECT_EQ(universal_view.dimensions().size(), 0);
  EXPECT_FALSE(universal_view.isOfType<TensorConstView3i>());
  EXPECT_FALSE(universal_view.tryGet<TensorConstView3i>());
  EXPECT_FALSE(universal_view.isOfType<TensorConstView3f>());
  EXPECT_FALSE(universal_view.tryGet<TensorConstView3f>());
}

TEST(UniversalTensor, DeviceEmpty) {
  CudaUniversalTensorConstView universal_view;
  EXPECT_EQ(universal_view.element_type(), ElementType::kUnknown);
  EXPECT_EQ(universal_view.rank(), 0);
  EXPECT_EQ(universal_view.dimensions().size(), 0);
  EXPECT_FALSE(universal_view.isOfType<CudaTensorConstView3i>());
  EXPECT_FALSE(universal_view.tryGet<CudaTensorConstView3i>());
  EXPECT_FALSE(universal_view.isOfType<CudaTensorConstView3f>());
  EXPECT_FALSE(universal_view.tryGet<CudaTensorConstView3f>());
}

TEST(UniversalTensor, Tensor3i) {
  Tensor3i tensor(17, 13, 29);
  tensor(2, 3, 5) = 19;

  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.element_type(), ElementType::kInt32);
  EXPECT_EQ(universal_view.rank(), 3);
  ASSERT_EQ(universal_view.dimensions().size(), 3);
  EXPECT_EQ(universal_view.dimensions()[0], 17);
  EXPECT_EQ(universal_view.dimensions()[1], 13);
  EXPECT_EQ(universal_view.dimensions()[2], 29);
  EXPECT_TRUE(universal_view.isOfType<TensorConstView3i>());
  EXPECT_TRUE(universal_view.tryGet<TensorConstView3i>());
  EXPECT_FALSE(universal_view.isOfType<TensorConstView3f>());
  EXPECT_FALSE(universal_view.tryGet<TensorConstView3f>());

  const auto view = universal_view.get<TensorConstView3i>();
  EXPECT_EQ(view(2, 3, 5), 19);
}

TEST(UniversalTensor, Tensor3f) {
  Tensor3f tensor(17, 13, 29);
  tensor(2, 3, 5) = 19.4f;

  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.element_type(), ElementType::kFloat32);
  EXPECT_EQ(universal_view.rank(), 3);
  ASSERT_EQ(universal_view.dimensions().size(), 3);
  EXPECT_EQ(universal_view.dimensions()[0], 17);
  EXPECT_EQ(universal_view.dimensions()[1], 13);
  EXPECT_EQ(universal_view.dimensions()[2], 29);
  EXPECT_FALSE(universal_view.isOfType<TensorConstView3i>());
  EXPECT_FALSE(universal_view.tryGet<TensorConstView3i>());

  EXPECT_TRUE(universal_view.isOfType<TensorConstView3f>());
  EXPECT_TRUE(universal_view.tryGet<TensorConstView3f>());

  const auto view = universal_view.get<TensorConstView3f>();
  EXPECT_EQ(view(2, 3, 5), 19.4f);
}

TEST(UniversalTensor, CompatibleDimensions) {
  // Tensor with no leading or trailing ones in the dimensions
  Tensor3f tensor(2, 3, 4);
  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.rank(), 3);
  EXPECT_FALSE(universal_view.isRankCompatible<0>());
  EXPECT_FALSE(universal_view.isRankCompatible<1>());
  EXPECT_FALSE(universal_view.isRankCompatible<2>());
  EXPECT_TRUE(universal_view.isRankCompatible<3>());
  EXPECT_TRUE(universal_view.isRankCompatible<4>());
}

TEST(UniversalTensor, CompatibleDimensionsLeadingOnes) {
  // Tensor with leading ones in the dimensions
  Tensor4f tensor(1, 1, 3, 4);
  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.rank(), 4);
  ASSERT_EQ(universal_view.dimensions().size(), 4);
  EXPECT_FALSE(universal_view.isRankCompatible<0>());
  EXPECT_FALSE(universal_view.isRankCompatible<1>());
  EXPECT_TRUE(universal_view.isRankCompatible<2>());
  EXPECT_TRUE(universal_view.isRankCompatible<3>());
  EXPECT_TRUE(universal_view.isRankCompatible<4>());
  EXPECT_TRUE(universal_view.isRankCompatible<5>());
}

TEST(UniversalTensor, CompatibleDimensionsLeadingAndTrailingOnes) {
  // Tensor with leading and trailing ones in the dimensions
  Tensor4f tensor(1, 3, 4, 1);
  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.rank(), 4);
  EXPECT_FALSE(universal_view.isRankCompatible<0>());
  EXPECT_FALSE(universal_view.isRankCompatible<1>());
  EXPECT_FALSE(universal_view.isRankCompatible<2>());
  EXPECT_TRUE(universal_view.isRankCompatible<3>());
  EXPECT_TRUE(universal_view.isRankCompatible<4>());
}

TEST(UniversalTensor, CompatibleDimensionsTrailingOnes) {
  // Tensor with trailing ones in the dimensions
  Tensor4f tensor(3, 4, 1, 1);
  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.rank(), 4);
  EXPECT_FALSE(universal_view.isRankCompatible<0>());
  EXPECT_FALSE(universal_view.isRankCompatible<1>());
  EXPECT_FALSE(universal_view.isRankCompatible<2>());
  EXPECT_FALSE(universal_view.isRankCompatible<3>());
  EXPECT_TRUE(universal_view.isRankCompatible<4>());
}

TEST(UniversalTensor, CompatibleDimensionsMiddleOnes) {
  // Tensor with ones in the middle of the dimensions but not at the beginning or end
  Tensor4f tensor(3, 1, 1, 4);
  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.rank(), 4);
  EXPECT_FALSE(universal_view.isRankCompatible<0>());
  EXPECT_FALSE(universal_view.isRankCompatible<1>());
  EXPECT_FALSE(universal_view.isRankCompatible<2>());
  EXPECT_FALSE(universal_view.isRankCompatible<3>());
  EXPECT_TRUE(universal_view.isRankCompatible<4>());
}

TEST(UniversalTensor, CompatibleDimensionsOnlyOnes) {
  // Tensor with all ones as dimensions
  Tensor3f tensor(1, 1, 1);
  CpuUniversalTensorConstView universal_view(tensor.const_view());
  EXPECT_EQ(universal_view.rank(), 3);
  EXPECT_FALSE(universal_view.isRankCompatible<0>());
  EXPECT_TRUE(universal_view.isRankCompatible<1>());
  EXPECT_TRUE(universal_view.isRankCompatible<2>());
  EXPECT_TRUE(universal_view.isRankCompatible<3>());
}

}  // namespace isaac
