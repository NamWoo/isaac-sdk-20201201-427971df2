/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/math/fast_running_median.hpp"

#include <random>

#include "gtest/gtest.h"

namespace isaac {
namespace math {

TEST(FastRunningMedian, normal_usage) {
  std::mt19937 rng(1337);
  FastRunningMedian<double, 100> frm;
  std::uniform_real_distribution<double> obs_dis(4.0, 6.0);
  // Initialize with some value
  for (size_t i = 0; i < 100; ++i) {
    frm.add(obs_dis(rng));
  }
  // make sure the statistics are within reasonable range
  EXPECT_NEAR(frm.min(), 4.0, 0.3);
  EXPECT_NEAR(frm.percentile(0.1), 4.2, 0.3);
  EXPECT_NEAR(frm.median(), 5.0, 0.3);
  EXPECT_NEAR(frm.percentile(0.9), 5.8, 0.3);
  EXPECT_NEAR(frm.max(), 6.0, 0.3);
}

TEST(FastRunningMedian, hundred_samples_complete) {
  FastRunningMedian<double, 101> frm;
  for (size_t i = 0; i <= 100; ++i) {
    frm.add(i);
  }

  ASSERT_DOUBLE_EQ(frm.min(), 0);
  ASSERT_DOUBLE_EQ(frm.median(), 50);
  ASSERT_DOUBLE_EQ(frm.max(), 100);
}

TEST(FastRunningMedian, hundred_samples) {
  FastRunningMedian<double> frm;
  for (size_t i = 0; i <= 100; ++i) {
    frm.add(i);
  }

  ASSERT_DOUBLE_EQ(frm.min(), 0);
  ASSERT_DOUBLE_EQ(frm.median(), 49);
  ASSERT_DOUBLE_EQ(frm.max(), 100);
}

TEST(FastRunningMedian, million_samples) {
  FastRunningMedian<double> frm;
  for (size_t i = 0; i <= 1000000; ++i) {
    frm.add(i / 10000);
  }

  ASSERT_DOUBLE_EQ(frm.min(), 0);
  ASSERT_DOUBLE_EQ(frm.median(), 49);
  ASSERT_DOUBLE_EQ(frm.max(), 100);
}

TEST(FastRunningMedian, zero_input) {
  FastRunningMedian<double> frm;
  ASSERT_DOUBLE_EQ(frm.min(), std::numeric_limits<double>::max());
  ASSERT_DOUBLE_EQ(frm.percentile(0.1), 0.0);
  ASSERT_DOUBLE_EQ(frm.median(), 0.0);
  ASSERT_DOUBLE_EQ(frm.percentile(0.9), 0.0);
  ASSERT_DOUBLE_EQ(frm.max(), std::numeric_limits<double>::lowest());
}

TEST(FastRunningMedian, less_input) {
  std::mt19937 rng(1337);
  FastRunningMedian<double, 200> frm;
  std::uniform_real_distribution<double> obs_dis(4.0, 6.0);
  // Initialize with some value less than the size of the sample array
  for (size_t i = 0; i < 100; ++i) {
    frm.add(obs_dis(rng));
  }
  // make sure the statistics are within reasonable range
  EXPECT_NEAR(frm.min(), 4.0, 0.3);
  EXPECT_NEAR(frm.percentile(0.1), 4.2, 0.3);
  EXPECT_NEAR(frm.median(), 5.0, 0.3);
  EXPECT_NEAR(frm.percentile(0.9), 5.8, 0.3);
  EXPECT_NEAR(frm.max(), 6.0, 0.3);
}

TEST(FastRunningMedian, known_array) {
  double arr[7] = {10.01, 10.1, 10.2, 10.3, 10.4, 10.5, 100.1};

  FastRunningMedian<double, 7> frm;
  for (size_t i = 0; i < 7; ++i) {
    frm.add(arr[i]);
  }

  ASSERT_DOUBLE_EQ(frm.min(), 10.01);
  ASSERT_DOUBLE_EQ(frm.percentile(0.1), 10.01);
  ASSERT_DOUBLE_EQ(frm.median(), 10.3);
  ASSERT_DOUBLE_EQ(frm.percentile(0.9), 10.5);
  ASSERT_DOUBLE_EQ(frm.max(), 100.1);
}

}  // namespace math
}  // namespace isaac