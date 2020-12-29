/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/assert.hpp"

#include <random>
#include <sstream>

#include "gtest/gtest.h"

TEST(Assert, Test1) {
  std::default_random_engine rng;
  std::uniform_int_distribution<int> coin(0, 1);

  ASSERT_DEATH(ASSERT(false, "omg"), "omg");

  const int n = 5 + coin(rng);
  for (int i=1; i<n; i++) {
    std::stringstream ss;
    ss << "bad math " << i;
    ASSERT_DEATH(ASSERT(0==i, "bad math %d", i), ss.str().c_str());
  }

  ASSERT(1==1, "should never happen");
}
