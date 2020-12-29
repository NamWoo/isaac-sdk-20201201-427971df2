/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/math/float16.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Float16, Basics) {
  float16 a(3.4), b(5);
  float16 c = a * b;
  c += 3;
  EXPECT_NEAR(c, 20.0, 0.05);
}

}  // namespace isaac
