/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/array/cpu_array.hpp"

#include <numeric>

#include "engine/core/byte.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(CpuArray, Create) {
  CpuArray<int> buffer(10);
  EXPECT_FALSE(buffer.empty());
  EXPECT_EQ(buffer.size(), 10);
  EXPECT_NE(buffer.begin(), nullptr);
  EXPECT_EQ(buffer.end(), buffer.begin() + buffer.size());
}

TEST(CpuArray, ReadWrite) {
  CpuArray<int> buffer(1024);
  std::iota(buffer.begin(), buffer.end(), 0);
  for (size_t i = 0; i < buffer.size(); i++) {
    EXPECT_EQ(buffer.begin()[i], static_cast<int>(i));
  }
}

TEST(CpuArray, ViewFromBuffer) {
  CpuArray<int> buffer(10);
  std::iota(buffer.begin(), buffer.end(), 0);
  CpuArrayView<int> view = buffer.view();
  EXPECT_FALSE(view.empty());
  EXPECT_EQ(view.size(), 10);
  EXPECT_NE(view.begin(), nullptr);
  for (size_t i = 0; i < buffer.size(); i++) {
    EXPECT_EQ(view.begin()[i], static_cast<int>(i));
  }
}

TEST(CpuArray, WriteViewReadBuffer) {
  CpuArray<int> buffer(10);
  *buffer.begin() = 3;
  CpuArrayView<int> view = buffer.view();
  EXPECT_EQ(*view.begin(), 3);
  *view.begin() = 7;
  EXPECT_EQ(*buffer.begin(), 7);
}

TEST(CpuArray, MoveBuffer1) {
  CpuArray<int> buffer(10);
  std::iota(buffer.begin(), buffer.end(), 0);

  CpuArray<int> other = std::move(buffer);
  EXPECT_TRUE(buffer.empty());
  EXPECT_FALSE(other.empty());
  EXPECT_EQ(other.size(), 10);
  for (size_t i = 0; i < other.size(); i++) {
    EXPECT_EQ(other.begin()[i], static_cast<int>(i));
  }
}

TEST(CpuArray, MoveBuffer2) {
  CpuArray<int> buffer_a(10);
  CpuArray<int> buffer_b(20);
  int* ptr = buffer_b.begin();
  buffer_a = std::move(buffer_b);
  EXPECT_FALSE(buffer_a.empty());
  EXPECT_EQ(buffer_a.size(), 20);
  EXPECT_EQ(buffer_a.begin(), ptr);
  EXPECT_TRUE(buffer_b.empty());
  EXPECT_EQ(buffer_b.size(), 0);
  EXPECT_EQ(buffer_b.begin(), nullptr);
}

TEST(CpuArray, CastView) {
  CpuArray<byte> buffer(16);
  CpuArrayView<byte> byte_view = buffer.view();
  {
    CpuArrayView<uint32_t> ui32_view = byte_view.reinterpret<uint32_t>();
    EXPECT_FALSE(ui32_view.empty());
    EXPECT_EQ(ui32_view.size(), 4);
    for (size_t i = 0; i < ui32_view.size(); i++) {
      ui32_view.begin()[i] = static_cast<int>(i);
    }
  }
  EXPECT_FALSE(byte_view.empty());
  for (size_t i = 0; i < byte_view.size(); i += 4) {
    byte_view.begin()[i] = static_cast<byte>(i);
  }
}

TEST(CpuArray, ReinterpretViewSizeMismatch) {
  CpuArray<byte> buffer(10);
  EXPECT_DEATH(CpuArrayView<float> view = buffer.view().reinterpret<float>(), ".*Incompatible.*");
}

TEST(CpuArray, MoveReinterpretBuffer) {
  CpuArray<int> buffer(10);
  CpuArray<float> float_buffer = buffer.move_reinterpret<float>();
  EXPECT_TRUE(buffer.empty());
  EXPECT_FALSE(float_buffer.empty());
  EXPECT_EQ(float_buffer.size(), 10);
}

TEST(CpuArray, MoveReinterpretMany) {
  constexpr size_t kBufferSize = 1'000'000;
  for (size_t i = 0; i < 50'000'000; i++) {
    CpuArray<int> buffer(kBufferSize);
    CpuArray<float> float_buffer = buffer.move_reinterpret<float>();
    EXPECT_TRUE(buffer.empty());
    EXPECT_FALSE(float_buffer.empty());
    EXPECT_EQ(float_buffer.size(), kBufferSize);
  }
}

TEST(CpuArray, ConstViewFromConstBuffer) {
  CpuArray<int> buffer(10);
  const CpuArray<int>& cref = buffer;
  ConstCpuArrayView<int> const_view = cref.const_view();
  EXPECT_FALSE(const_view.empty());
}

// -------------------------------------------------------------------------------------------------
// The following tests are all testing that certain expression do not compile. Please uncomment
// and check that the corresponding lines do not compile

/*
TEST(CpuArray, CopyBuffer) {
  CpuArray<int> buffer(10);
  CpuArray<int> other = buffer;  // should not compile
}

TEST(CpuArray, WriteConstView) {
  CpuArray<int> buffer(10);
  *buffer.begin() = 3;
  ConstCpuArrayView<int> view = buffer.const_view();
  EXPECT_EQ(*view.begin(), 3);
  *view.begin() = 7;  // should not compile
}

TEST(CpuArray, CastBuffer) {
  CpuArray<int> buffer(16);
  CpuArray<float> float_buffer = buffer.reinterpret<float>();  // should not compile
}

TEST(CpuArray, ViewFromConstBuffer) {
  CpuArray<int> buffer(10);
  const CpuArray<int>& cref = buffer;
  BufferView<int> non_const_view = cref.view();  // should not compile
}
*/

// End of "do not compile" tests.
// -------------------------------------------------------------------------------------------------

}  // namespace isaac
