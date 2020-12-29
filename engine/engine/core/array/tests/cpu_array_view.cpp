/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/array/cpu_array_view.hpp"

#include <cstdint>

#include "gtest/gtest.h"

namespace isaac {

TEST(CpuArrayView, Construct) {
  std::vector<uint16_t> buffer(8);
  CpuArrayView<uint16_t> view{buffer.data(), buffer.size()};
  EXPECT_FALSE(view.empty());
  EXPECT_EQ(view.size(), 8);
  EXPECT_EQ(view.begin(), buffer.data());
  EXPECT_EQ(view.end(), buffer.data() + 8);
}

TEST(CpuArrayView, Reinterpret) {
  std::vector<uint16_t> buffer(8);
  CpuArrayView<uint16_t> view{buffer.data(), buffer.size()};
  CpuArrayView<uint64_t> view2 = view.reinterpret<uint64_t>();
  EXPECT_FALSE(view2.empty());
  EXPECT_EQ(view2.size(), 2);
  EXPECT_EQ(view2.begin(), reinterpret_cast<uint64_t*>(buffer.data()));
}

TEST(CpuArrayView, ViewToConstView) {
  std::vector<uint16_t> buffer(8);
  CpuArrayView<uint16_t> view{buffer.data(), buffer.size()};
  ConstCpuArrayView<uint16_t> const_view = view;
  EXPECT_EQ(const_view.size(), 8);
  EXPECT_EQ(const_view.begin(), buffer.data());
  EXPECT_EQ(const_view.end(), buffer.data() + 8);
}

}  // namespace isaac
