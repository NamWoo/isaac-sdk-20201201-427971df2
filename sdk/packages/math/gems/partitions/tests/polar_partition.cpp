/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/partitions/polar_partition.hpp"

#include "gtest/gtest.h"

namespace isaac {
namespace partitions {

TEST(PolarPartition, Slice) {
  auto p = RegularPolarSlicePartition<double>{
    RegularIntervalPartition<double>::FromCellCount(DegToRad(-90.0), DegToRad(+90.0), 10),
    RegularIntervalPartition<double>::FromCellCount(0.0, 10.0, 20)
  };
  EXPECT_EQ(p.sector_partition.num_samples(), 11);
  EXPECT_EQ(p.range_partition.num_samples(), 21);

  auto cell = p.toNearest({0.0, -1.0});
  EXPECT_EQ(cell[0], 0);
  EXPECT_EQ(cell[1], 2);
  cell = p.toNearest({0.0, +2.0});
  EXPECT_EQ(cell[0], 10);
  EXPECT_EQ(cell[1], 4);
  cell = p.toNearest({3.0, 0.0});
  EXPECT_EQ(cell[0], 5);
  EXPECT_EQ(cell[1], 6);
}

}  // namespace partitions
}  // namespace isaac
