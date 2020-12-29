/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/time.hpp"

#include "gtest/gtest.h"

namespace isaac {

TEST(Core, TimeTest1) {
  double duration = 0.95;
  for (int i = 0; i < 10; i++) {
    const int64_t a = NowCount();
    Sleep(SecondsToNano(duration));
    const int64_t b = NowCount();
    const int64_t dt = b - a;
    EXPECT_NEAR(ToSeconds(dt), duration, 0.001);
    duration *= 0.5;
  }
}

TEST(Core, ParseDurationStringToSecond) {
  EXPECT_NEAR(*ParseDurationStringToSecond("1s"), 1.0f, 1e-6);
  EXPECT_NEAR(*ParseDurationStringToSecond("100s"), 100.0f, 1e-6);
  EXPECT_NEAR(*ParseDurationStringToSecond("1m"), 60.0f, 1e-6);
  EXPECT_NEAR(*ParseDurationStringToSecond("10m"), 600.0f, 1e-6);
  EXPECT_NEAR(*ParseDurationStringToSecond("1h"), 3600.0f, 1e-6);
  EXPECT_NEAR(*ParseDurationStringToSecond("10h"), 36000.0f, 1e-6);
}

TEST(Core, inverse_function) {
  for (int64_t i = 0; i < 2000000000000000ll; i = 1 + i*1.001) {
    EXPECT_EQ(SecondsToNano(ToSeconds(i)), i);
  }
}

}  // namespace isaac
