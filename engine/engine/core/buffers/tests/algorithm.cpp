/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/buffers/algorithm.hpp"

#include <algorithm>
#include <numeric>
#include <vector>

#include "gtest/gtest.h"

namespace isaac {

TEST(Algorithm, CopyArrayRawHost) {
  std::vector<int> range1(100);
  std::iota(range1.begin(), range1.end(), 0);

  std::vector<int> range2(100);
  CopyArrayRaw(range1.data(), BufferStorageMode::Host, range2.data(), BufferStorageMode::Host,
               range2.size() * sizeof(int));

  for (size_t i = 0; i < range1.size(); i++) {
    ASSERT_EQ(range1[i], range2[i]);
  }
}

TEST(Algorithm, CopyMatrixRawHost) {
  int rows = 120;
  int cols = 7;
  int stride1 = cols + 11;
  int stride2 = cols + 171;
  std::vector<int> range1(rows * stride1);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      range1[row * stride1 + col] = row * col;
    }
  }

  std::vector<int> range2(rows * stride2);
  CopyMatrixRaw(range1.data(), stride1 * sizeof(int), BufferStorageMode::Host, range2.data(),
                stride2 * sizeof(int), BufferStorageMode::Host, rows, cols * sizeof(int));

  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      ASSERT_EQ(range1[row * stride1 + col], range2[row * stride2 + col]);
    }
  }
}

}  // namespace isaac
