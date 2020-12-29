/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/partitions/regular_partition.hpp"

#include "gtest/gtest.h"

namespace isaac {
namespace partitions {

TEST(RegularPartition, ToBucket1) {
  const RegularPartition<double> partition(1.0);
  EXPECT_EQ(partition.toNearest(0.0), 0);
  EXPECT_EQ(partition.toNearest(0.3), 0);
  EXPECT_EQ(partition.toNearest(0.49), 0);
  EXPECT_EQ(partition.toNearest(0.51), 1);
  EXPECT_EQ(partition.toNearest(0.9999), 1);
  EXPECT_EQ(partition.toNearest(1.0), 1);
  EXPECT_EQ(partition.toNearest(1.49), 1);
  EXPECT_EQ(partition.toNearest(1.51), 2);
  EXPECT_EQ(partition.toNearest(2.0), 2);
  EXPECT_EQ(partition.toNearest(3.4), 3);
  EXPECT_EQ(partition.toNearest(3.5), 4);
  EXPECT_EQ(partition.toNearest(-3.0), -3);
  EXPECT_EQ(partition.toNearest(-2.6), -3);
  EXPECT_EQ(partition.toNearest(-2.4), -2);
  EXPECT_EQ(partition.toNearest(-1.5), -2);
  EXPECT_EQ(partition.toNearest(-1.0), -1);
  EXPECT_EQ(partition.toNearest(-0.9999), -1);
  EXPECT_EQ(partition.toNearest(-0.51), -1);
  EXPECT_EQ(partition.toNearest(-0.49), 0);
  EXPECT_EQ(partition.toNearest(-0.0001), 0);
}

TEST(RegularPartition, ToBucket2) {
  const RegularPartition<double> partition(2.0);
  EXPECT_EQ(partition.toNearest(-3.01), -2);
  EXPECT_EQ(partition.toNearest(-2.99), -1);
  EXPECT_EQ(partition.toNearest(-2.00), -1);
  EXPECT_EQ(partition.toNearest(-1.01), -1);
  EXPECT_EQ(partition.toNearest(-0.99), 0);
  EXPECT_EQ(partition.toNearest(0.0), 0);
  EXPECT_EQ(partition.toNearest(0.99), 0);
  EXPECT_EQ(partition.toNearest(1.01), 1);
  EXPECT_EQ(partition.toNearest(2.99), 1);
  EXPECT_EQ(partition.toNearest(3.01), 2);
}

TEST(RegularPartition, ToBucketWithRemainder) {
  const RegularPartition<double> partition(1.0);
  double remainder;
  EXPECT_EQ(partition.toLinear(3.5, remainder), 3);
  EXPECT_NEAR(remainder, 0.5, 1e-9);
  EXPECT_EQ(partition.toLinear(3.2, remainder), 3);
  EXPECT_NEAR(remainder, 0.2, 1e-9);
  EXPECT_EQ(partition.toLinear(-0.4, remainder), -1);
  EXPECT_NEAR(remainder, 0.6, 1e-9);
  EXPECT_EQ(partition.toLinear(-3.1, remainder), -4);
  EXPECT_NEAR(remainder, 0.9, 1e-9);
}

TEST(RegularPartition, ToCartestian1) {
  const RegularPartition<double> partition(1.0);
  EXPECT_EQ(partition.cellCenter(-2), -2.0);
  EXPECT_EQ(partition.cellCenter(0), 0.0);
  EXPECT_EQ(partition.cellCenter(1), 1.0);
  EXPECT_EQ(partition.cellLower(1), 0.5);
  EXPECT_EQ(partition.cellLower(-1), -1.5);
  EXPECT_EQ(partition.cellUpper(1), 1.5);
  EXPECT_EQ(partition.cellUpper(-1), -0.5);
}

TEST(RegularPartition, ToCartestian2) {
  const RegularPartition<double> partition(1.5);
  EXPECT_EQ(partition.cellCenter(-2), -3.0);
  EXPECT_EQ(partition.cellCenter(0), 0.0);
  EXPECT_EQ(partition.cellCenter(1), 1.5);
  EXPECT_EQ(partition.cellLower(1), 0.75);
  EXPECT_EQ(partition.cellLower(-1), -2.25);
  EXPECT_EQ(partition.cellUpper(1), 2.25);
  EXPECT_EQ(partition.cellUpper(-1), -0.75);
}

}  // namespace partitions
}  // namespace isaac
