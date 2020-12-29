/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>
#include <vector>

#include "engine/core/logger.hpp"
#include "engine/gems/algorithm/bresenham.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Bresenham, TestReturn) {
  EXPECT_TRUE(Bresenham(Vector2i{0,0}, Vector2i{10,10}, [](int x, int y) { return true; }));
  EXPECT_FALSE(Bresenham(Vector2i{0,0}, Vector2i{10,10}, [](int x, int y) { return false; }));
}

int CountBresenham(const Vector2i& start, const Vector2i& end) {
  int count = 0;
  Bresenham(start, end, [&](int,int) { count++; return true; });
  return count;
}

TEST(Bresenham, Count) {
  EXPECT_EQ(CountBresenham(Vector2i{0,0}, Vector2i{0,0}), 1);
  EXPECT_EQ(CountBresenham(Vector2i{0,0}, Vector2i{10,0}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{0,0}, Vector2i{-10,0}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{10,0}, Vector2i{0,0}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{-10,0}, Vector2i{0,0}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{0,0}, Vector2i{0,10}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{0,0}, Vector2i{0,-10}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{0,10}, Vector2i{0,0}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{0,-10}, Vector2i{0,0}), 11);
  EXPECT_EQ(CountBresenham(Vector2i{0,0}, Vector2i{10,10}), 11);
}

}  // namespace isaac
