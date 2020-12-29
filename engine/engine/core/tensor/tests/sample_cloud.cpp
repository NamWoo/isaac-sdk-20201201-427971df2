/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/tensor/sample_cloud.hpp"

#include "gtest/gtest.h"

namespace isaac {

TEST(SampleCloud, Creation) {
  SampleCloud1ub pc1(3);
  EXPECT_EQ(pc1.rank(), 1);
  EXPECT_EQ(pc1.dimensions()[0], 3);
  EXPECT_EQ(pc1.data().size(), 3);
  SampleCloud2d pc2;
  EXPECT_EQ(pc2.rank(), 2);
  EXPECT_EQ(pc2.dimensions()[0], 0);
  EXPECT_EQ(pc2.dimensions()[1], 2);
  EXPECT_EQ(pc2.data().size(), 0);
  SampleCloud3f pc3(10);
  EXPECT_EQ(pc3.rank(), 2);
  EXPECT_EQ(pc3.dimensions()[0], 10);
  EXPECT_EQ(pc3.dimensions()[1], 3);
  EXPECT_EQ(pc3.data().size(), 120);
  SampleCloud4i pc4(100);
  EXPECT_EQ(pc4.rank(), 2);
  EXPECT_EQ(pc4.dimensions()[0], 100);
  EXPECT_EQ(pc4.dimensions()[1], 4);
  EXPECT_EQ(pc4.data().size(), 1600);
}

}  // namespace isaac
