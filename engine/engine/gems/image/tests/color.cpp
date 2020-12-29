/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/image/image.hpp"

#include "engine/gems/image/color.hpp"

#include "gtest/gtest.h"

namespace isaac {

TEST(color, BlackWhiteColorGradient) {
  auto gradient = BlackWhiteColorGradient();
  Pixel3ub v = gradient(0.0);
  EXPECT_EQ(v[0], 0);
  EXPECT_EQ(v[1], 0);
  EXPECT_EQ(v[2], 0);
  v = gradient(0.5);
  EXPECT_EQ(v[0], 128);
  EXPECT_EQ(v[1], 128);
  EXPECT_EQ(v[2], 128);
  v = gradient(1.0);
  EXPECT_EQ(v[0], 255);
  EXPECT_EQ(v[1], 255);
  EXPECT_EQ(v[2], 255);
}

}  // namespace isaac
