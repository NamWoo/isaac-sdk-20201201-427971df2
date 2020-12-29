/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/tensor/element_type.hpp"

#include "gtest/gtest.h"

namespace isaac {

TEST(ElementType, Types) {
  EXPECT_EQ(GetElementType<char>(), ElementType::kInt8);
  EXPECT_EQ(GetElementType<unsigned char>(), ElementType::kUInt8);
  EXPECT_EQ(GetElementType<int>(), ElementType::kInt32);
  EXPECT_EQ(GetElementType<short>(), ElementType::kInt16);
  EXPECT_EQ(GetElementType<unsigned>(), ElementType::kUInt32);
  EXPECT_EQ(GetElementType<float>(), ElementType::kFloat32);
  EXPECT_EQ(GetElementType<double>(), ElementType::kFloat64);
}

}  // namespace isaac
