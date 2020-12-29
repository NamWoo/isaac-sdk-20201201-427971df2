/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/partitions/plane_partition.hpp"

#include "gtest/gtest.h"
#include "packages/math/gems/partitions/regular_interval_partition.hpp"

namespace isaac {
namespace partitions {

TEST(PlanePartition, Slice) {
  PlanePartition<double, int, RegularIntervalPartition, RegularIntervalPartition> partition;
  partition.partition<0>() =
      RegularIntervalPartition<double>::FromCellCount(-2.0, +2.0, 2);
  partition.partition<1>() =
      RegularIntervalPartition<double>::FromCellCount(1.0, 4.0, 3);

  auto cell = partition.toNearest({0.3, -1.2});
  EXPECT_EQ(cell[0], 1);
  EXPECT_EQ(cell[1], -2);
}

}  // namespace partitions
}  // namespace isaac
