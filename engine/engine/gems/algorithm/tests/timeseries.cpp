/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <cmath>

#include "engine/gems/algorithm/timeseries.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Timeseries, Empty) {
  Timeseries<int, double> h;
  EXPECT_TRUE(h.empty());
  EXPECT_EQ(h.size(), 0);
  EXPECT_DEATH(h.oldest(), ".*");
  EXPECT_DEATH(h.youngest(), ".*");
  EXPECT_DEATH(h.at(1), ".*");
  EXPECT_DEATH(h.state(1), ".*");
  EXPECT_EQ(h.lower_index(1.3), -1);
  EXPECT_EQ(h.upper_index(1.3), 0);
  EXPECT_EQ(h.interpolate_2_index(1.3), -1);
}

TEST(Timeseries, Basics) {
  Timeseries<int, double> h;
  h.insert(1.3, -1);
  h.insert(1.8, 2);
  h.insert(2.1, 5);
  EXPECT_FALSE(h.empty());
  EXPECT_EQ(h.size(), 3);
  EXPECT_EQ(h.oldest().stamp, 1.3);
  EXPECT_EQ(h.oldest().state, -1);
  EXPECT_EQ(h.youngest().stamp, 2.1);
  EXPECT_EQ(h.youngest().state, 5);
  EXPECT_EQ(h.at(1).stamp, 1.8);
  EXPECT_EQ(h.at(1).state, 2);
  EXPECT_EQ(h.state(1), 2);
  EXPECT_EQ(h.lower_index(0.5), -1);
  EXPECT_EQ(h.lower_index(1.5), 0);
  EXPECT_EQ(h.lower_index(3.5), 2);
  EXPECT_EQ(h.upper_index(0.5), 0);
  EXPECT_EQ(h.upper_index(1.5), 1);
  EXPECT_EQ(h.upper_index(3.5), 3);
  auto interp = [](double p, int a, int b) { return std::round(a + p * (b - a)); };
  auto maybe = h.interpolate_2_p(1.9, interp);
  ASSERT_TRUE((bool)maybe);
  EXPECT_EQ(*maybe, 3);
}

}  // namespace isaac
