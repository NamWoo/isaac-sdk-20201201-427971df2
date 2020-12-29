/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/geometry/transform_to_plane.hpp"

#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/gems/geometry/pinhole.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace geometry {

TEST(TransformToPlane, 2d_pose_pinhole) {

  const int N_test = 3;

  // 2d pixel points
  Matrix<double, N_test, 2> pixels;
  pixels << 150, 150,
            100, 150,
            50, 150;

  // construct the pinhole camera and the pose
  const auto& pinhole = Pinhole<double>::FromHorizontalFieldOfView(
      200, 300, DegToRad(90.0));

  const Pose3d pose{
      SO3d::FromAngleAxis(-Pi<double> / 180.0 * 90.0, {1.0, 0.0, 0.0}),
      Vector3d(0.0, 0.0, 1.0)
  };

  // correct output points
  Matrix<double, N_test, 3> correct;
  correct << 0.0, 1.5, 0.0,
             0.0, 0.0, 0.0,  // we will not check this row as there is no projection
             0.0, -1.5, 0.0;

  // correct optional returns
  const bool correct_optionals[3] = {true, false, false};

  // test against the correct output
  for(int i = 0; i < N_test; i ++) {
    const auto& pixel = pixels.block(i, 0, 1, 2);
    if (const auto res = TransformToPlane<double>(pixel.transpose(), pinhole, pose, 0.5)) {
      ASSERT(correct_optionals[i], "Fail to detect the error");
      for(int j = 0; j < 2; j ++)
        EXPECT_NEAR((*res)(j), correct(i, j), 1e-6);
    } else {
      ASSERT(!correct_optionals[i], "Fail to detect the error");
    }
  }
}

}  // namespace geometry
}  // namespace isaac
