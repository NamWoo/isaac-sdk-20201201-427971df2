/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/buffers/buffer.hpp"

#include <cstdint>

#include "gtest/gtest.h"

namespace isaac {

TEST(AlignedBuffer, MallocConstructEmpty) {
  CpuBuffer buffer;
  EXPECT_EQ(buffer.size(), 0);
  EXPECT_EQ(buffer.pointer(), nullptr);
}

TEST(AlignedBuffer, MallocConstruct) {
  CpuBuffer buffer(200);
  EXPECT_EQ(buffer.size(), 200);
  EXPECT_NE(buffer.pointer(), nullptr);
}

TEST(AlignedBuffer, Move) {
  CpuBuffer buffer(450);
  EXPECT_EQ(buffer.size(), 450);
  EXPECT_NE(buffer.pointer(), nullptr);

  CpuBuffer buffer2 = std::move(buffer);
  EXPECT_EQ(buffer2.size(), 450);
  EXPECT_NE(buffer2.pointer(), nullptr);
}

}  // namespace isaac
