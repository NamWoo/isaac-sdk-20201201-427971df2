/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/partitions/regular_interval_partition.hpp"

#include "gtest/gtest.h"

namespace isaac {
namespace partitions {

TEST(RegularIntervalPartition, FromCount) {
  auto p1 = RegularIntervalPartition<double>::FromCellCount(0.0, 1.0, 1);
  EXPECT_EQ(p1.num_cells(), 1);
  EXPECT_EQ(p1.min(), 0.0);
  EXPECT_EQ(p1.max(), 1.0);
  EXPECT_EQ(p1.delta(), 1.0);
  auto p2 = RegularIntervalPartition<double>::FromCellCount(-1.0, 2.0, 2);
  EXPECT_EQ(p2.num_cells(), 2);
  EXPECT_EQ(p2.min(), -1.0);
  EXPECT_EQ(p2.max(), 2.0);
  EXPECT_EQ(p2.delta(), 1.5);
}

TEST(RegularIntervalPartition, FromDelta) {
  auto p1 = RegularIntervalPartition<double>::FromDelta(0.0, 1.0, 0.5);
  EXPECT_EQ(p1.num_cells(), 2);
  EXPECT_EQ(p1.min(), 0.0);
  EXPECT_EQ(p1.max(), 1.0);
  EXPECT_EQ(p1.delta(), 0.5);
  auto p2 = RegularIntervalPartition<double>::FromDelta(-1.0, 2.0, 0.5);
  EXPECT_EQ(p2.num_cells(), 6);
  EXPECT_EQ(p2.min(), -1.0);
  EXPECT_EQ(p2.max(), 2.0);
  EXPECT_EQ(p2.delta(), 0.5);
}

TEST(RegularIntervalPartition, ToBucketUnit) {
  auto p2 = RegularIntervalPartition<double>::FromCellCount(0.0, 1.0, 1);
  EXPECT_EQ(p2.toNearest(-0.51), -1);
  EXPECT_EQ(p2.toNearest(-0.49), 0);
  EXPECT_EQ(p2.toNearest(0.0), 0);
  EXPECT_EQ(p2.toNearest(0.49), 0);
  EXPECT_EQ(p2.toNearest(0.51), 1);
  EXPECT_EQ(p2.toNearest(1.0), 1);
  EXPECT_EQ(p2.toNearest(1.49), 1);
  EXPECT_EQ(p2.toNearest(1.51), 2);
}

TEST(RegularIntervalPartition, ToBucket) {
  //    |     S0     |     S1    |    S2   |
  //  -1.75  -1.0  -0.25  0.5  1.25  2.0  2.75
  auto p2 = RegularIntervalPartition<double>::FromCellCount(-1.0, 2.0, 2);
  EXPECT_EQ(p2.toNearest(-1.9), -1);
  EXPECT_EQ(p2.toNearest(-1.5), 0);
  EXPECT_EQ(p2.toNearest(-1.0), 0);
  EXPECT_EQ(p2.toNearest(-0.5), 0);
  EXPECT_EQ(p2.toNearest(0.5), 1);
  EXPECT_EQ(p2.toNearest(0.6), 1);
  EXPECT_EQ(p2.toNearest(2.0), 2);
  EXPECT_EQ(p2.toNearest(3.4), 3);
  EXPECT_EQ(p2.toNearest(3.5), 3);
  double remainder;
  EXPECT_EQ(p2.toLinear(3.5, remainder), 3);
  EXPECT_NEAR(remainder, 0.0, 1e-9);
  EXPECT_EQ(p2.toLinear(3.2, remainder), 2);
  EXPECT_NEAR(remainder, 0.8, 1e-9);
  EXPECT_EQ(p2.toLinear(-3.1, remainder), -2);
  EXPECT_NEAR(remainder, 0.6, 1e-9);
}

TEST(RegularIntervalPartition, BucketCenter) {
  auto p2 = RegularIntervalPartition<double>::FromCellCount(-1.0, 2.0, 2);
  EXPECT_EQ(p2.cellCenter(-2), -4.0);
  EXPECT_EQ(p2.cellCenter(0), -1.0);
  EXPECT_EQ(p2.cellCenter(1), 0.5);
}

}  // namespace partitions
}  // namespace isaac
