/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/buffers/shared_buffer.hpp"

#include <cstdint>

#include "gtest/gtest.h"

namespace isaac {

TEST(SharedBuffer, HostToCuda) {
  constexpr size_t kSize = 100;
  CpuBuffer buffer(kSize);
  SharedBuffer shared(std::move(buffer));
  EXPECT_EQ(shared.host_buffer().size(), kSize);
  EXPECT_EQ(shared.cuda_buffer().size(), kSize);
}

TEST(SharedBuffer, CudaToHost) {
  constexpr size_t kSize = 100;
  CudaBuffer buffer(kSize);
  SharedBuffer shared(std::move(buffer));
  EXPECT_EQ(shared.cuda_buffer().size(), kSize);
  EXPECT_EQ(shared.host_buffer().size(), kSize);
}

}  // namespace isaac
