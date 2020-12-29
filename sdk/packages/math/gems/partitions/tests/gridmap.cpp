/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/partitions/grid_map.hpp"

#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace partitions {

TEST(GridMap, Basics) {
  GridMap<double> gm;

  gm.setCellSize(0.25);
  EXPECT_EQ(gm.cell_size(), 0.25);

  const Pose2d world_T_map_actual{SO2d::FromAngle(DegToRad(90.0)), Vector2d{4.0, -3.0}};
  gm.setWorldTMap(world_T_map_actual);
  ISAAC_EXPECT_POSE_NEAR(gm.world_T_map(), world_T_map_actual, 1e-12);
  ISAAC_EXPECT_POSE_NEAR(gm.map_T_world(), world_T_map_actual.inverse(), 1e-6);

  gm.data.resize(60, 80);
  EXPECT_EQ(gm.rows(), 60);
  EXPECT_EQ(gm.cols(), 80);

  ISAAC_EXPECT_VEC_NEAR(gm.range_max(), (Vector2d{15.0, 20.0}), 1e-9);

  EXPECT_FALSE(gm.isInRange(-1, -1));
  EXPECT_TRUE(gm.isInRange(0, 0));
  EXPECT_TRUE(gm.isInRange(59, 79));
  EXPECT_FALSE(gm.isInRange(60, 80));
}

TEST(GridMap, MapToCell) {
  GridMap<double> gm;
  gm.setCellSize(0.25);
  gm.setWorldTMap(Pose2d::Identity());
  gm.data.resize(60, 80);

  const Vector2i p_grid{14, 9};
  const Vector2d p_map_mid = gm.cellCenterToMap(p_grid);
  ISAAC_EXPECT_VEC_NEAR(p_map_mid, (Vector2d{3.50, 2.25}), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(gm.mapToCell(Vector2d{3.500, 2.250}), (Vector2i{14, 9}), 0);
  ISAAC_EXPECT_VEC_NEAR(gm.mapToCell(Vector2d{3.500, 2.374}), (Vector2i{14, 9}), 0);
  ISAAC_EXPECT_VEC_NEAR(gm.mapToCell(Vector2d{3.624, 2.250}), (Vector2i{14, 9}), 0);
  ISAAC_EXPECT_VEC_NEAR(gm.mapToCell(Vector2d{3.626, 2.375}), (Vector2i{15, 10}), 0);

  Vector2d fractional;
  Vector2i integral;
  integral = gm.mapToCell(Vector2d{3.500, 2.250}, fractional);
  ISAAC_EXPECT_VEC_NEAR(integral, (Vector2d{14, 9}), 0);
  ISAAC_EXPECT_VEC_NEAR(fractional, (Vector2d{0, 0}), 1e-9);
  integral = gm.mapToCell(Vector2d{3.375, 2.250}, fractional);
  ISAAC_EXPECT_VEC_NEAR(integral, (Vector2d{13, 9}), 0);
  ISAAC_EXPECT_VEC_NEAR(fractional, (Vector2d{0.5, 0}), 1e-9);
  integral = gm.mapToCell(Vector2d{3.500, 2.375}, fractional);
  ISAAC_EXPECT_VEC_NEAR(integral, (Vector2d{14, 9}), 0);
  ISAAC_EXPECT_VEC_NEAR(fractional, (Vector2d{0.0, 0.5}), 1e-9);
}

}  // namespace partitions
}  // namespace isaac
