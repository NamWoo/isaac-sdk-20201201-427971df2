/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/partitions/ring_partition.hpp"

#include "gtest/gtest.h"

namespace isaac {
namespace partitions {

TEST(RingPartition, AngularCreate) {
  auto p = RingPartition<double>::Angular(20);

  EXPECT_EQ(p.num_cells(), 20);
  EXPECT_EQ(RadToDeg(p.length()), 360.0);
  EXPECT_EQ(RadToDeg(p.delta()), 18.0);
}

TEST(RingPartition, AngularToNearest) {
  auto p = RingPartition<double>::Angular(20);

  EXPECT_EQ(p.toNearest(DegToRad(10.0)), 1);
  EXPECT_EQ(p.toNearest(DegToRad(8.0)), 0);
  EXPECT_EQ(p.toNearest(DegToRad(-8.0)), 0);
  EXPECT_EQ(p.toNearest(DegToRad(-10.0)), 19);
  EXPECT_EQ(p.toNearest(DegToRad(-18.0)), 19);
  EXPECT_EQ(p.toNearest(DegToRad(-26.0)), 19);
  EXPECT_EQ(p.toNearest(DegToRad(-29.0)), 18);
}

TEST(RingPartition, AngularToCell) {
  auto p = RingPartition<double>::Angular(20);

  double remainder;
  EXPECT_EQ(p.toLinear(DegToRad(20.0), remainder), 1);
  EXPECT_NEAR(remainder, 2.0/18.0, 1e-9);
  EXPECT_EQ(p.toLinear(DegToRad(-20.0), remainder), 18);
  EXPECT_NEAR(remainder, 16.0/18.0, 1e-9);

  EXPECT_EQ(p.cellCenter(-100), DegToRad(0.0));
  EXPECT_EQ(p.cellCenter(-10), DegToRad(180.0));
  EXPECT_EQ(p.cellCenter(-5), DegToRad(270.0));
  EXPECT_EQ(p.cellCenter(0), DegToRad(0.0));
  EXPECT_EQ(p.cellCenter(5), DegToRad(90.0));
  EXPECT_EQ(p.cellCenter(10), DegToRad(180.0));
  EXPECT_EQ(p.cellCenter(100), DegToRad(0.0));
}

}  // namespace partitions
}  // namespace isaac
