/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/epsilon.hpp"

#include <iostream>

#include "gtest/gtest.h"

namespace isaac {

TEST(Epsilon, MachineEpsilon) {
  EXPECT_GT(MachineEpsilon<float>, 0.0f);
  EXPECT_GT(MachineEpsilon<double>, 0.0);
  EXPECT_EQ(1.0f + MachineEpsilon<float>, 1.0f);
  EXPECT_EQ(1.0 + MachineEpsilon<double>, 1.0);
}

TEST(Epsilon, IsAlmostZero) {
  EXPECT_TRUE(IsAlmostZero(0.0f));
  EXPECT_TRUE(IsAlmostZero(-0.0f));
  EXPECT_TRUE(IsAlmostZero(MachineEpsilon<float>));
  EXPECT_TRUE(IsAlmostZero(-MachineEpsilon<float>));
  EXPECT_FALSE(IsAlmostZero(0.0001f));
  EXPECT_FALSE(IsAlmostZero(0.1f));
  EXPECT_TRUE(IsAlmostZero(0.0));
  EXPECT_TRUE(IsAlmostZero(-0.0));
  EXPECT_TRUE(IsAlmostZero(MachineEpsilon<double>));
  EXPECT_TRUE(IsAlmostZero(-MachineEpsilon<float>));
  EXPECT_FALSE(IsAlmostZero(0.00000001));
  EXPECT_FALSE(IsAlmostZero(0.1));
}

TEST(Epsilon, IsAlmostEqualRelative) {
  EXPECT_NE(67329.234f, 67329.242f);
  EXPECT_TRUE(IsAlmostEqualRelative(67329.234f, 67329.242f));
}

TEST(Epsilon, IsAlmostEqualUlps) {
  EXPECT_TRUE(IsAlmostEqualUlps(67329.234f, 67329.242f));
}

}  // namespace isaac
