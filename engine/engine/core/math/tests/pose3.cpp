/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/math/pose3.hpp"

#include <vector>

#include "engine/core/constants.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Pose3, Size) {
  EXPECT_EQ(sizeof(Pose3<long double>), 7 * sizeof(long double));
  EXPECT_GE(sizeof(Pose3d), 7 * sizeof(double));  // 64 instead of 56 bytes
  EXPECT_GE(sizeof(Pose3f), 7 * sizeof(float));  // 32 instead of 28 bytes
}

TEST(Pose3, MemoryLayout) {
  const Pose3d test{SO3d::FromQuaternion(Quaterniond(-0.1, 0.3, 0.4, 0.5)),
                    Vector3d{-0.7, 1.6, 0.6}};
  const double* pointer = reinterpret_cast<const double*>(&test);
  // Eigen storage order is x-y-z-w (although constructor takes w first)
  EXPECT_EQ(test.rotation.quaternion().x(), pointer[0]);
  EXPECT_EQ(test.rotation.quaternion().y(), pointer[1]);
  EXPECT_EQ(test.rotation.quaternion().z(), pointer[2]);
  EXPECT_EQ(test.rotation.quaternion().w(), pointer[3]);
  EXPECT_EQ(test.translation.x(), pointer[4]);
  EXPECT_EQ(test.translation.y(), pointer[5]);
  EXPECT_EQ(test.translation.z(), pointer[6]);
}

TEST(Pose3, composition) {
  Pose3d pose1{SO3d::FromAngleAxis(Pi<double>/2, {0.0, 0.0, 1.0}), Vector3d(2.0, 0.0, 7.0)};
  Pose3d pose2{SO3d::FromAngleAxis(Pi<double>/4, {0.0, 0.0, 1.0}), Vector3d(3.0, 0.0, 15.0)};
  Pose3d comp1 = pose1 * pose2;
  Pose3d comp2 = pose2 * pose1;
  EXPECT_NEAR(comp1.rotation.angle(), 3.0 * M_PI / 4.0, 1e-7);
  EXPECT_NEAR(comp1.translation.x(), 2.0, 1e-7);
  EXPECT_NEAR(comp1.translation.y(), 3.0, 1e-7);
  EXPECT_NEAR(comp1.translation.z(), 22.0, 1e-7);
  EXPECT_NEAR(comp2.rotation.angle(), 3.0 * M_PI / 4.0, 1e-7);
  EXPECT_NEAR(comp2.translation.x(), std::sqrt(2) + 3.0, 1e-7);
  EXPECT_NEAR(comp2.translation.y(), std::sqrt(2), 1e-7);
  EXPECT_NEAR(comp2.translation.z(), 22.0, 1e-7);
}

TEST(Pose3, inverse) {
  Pose3d pose1{SO3d::FromAngleAxis(1.23456, {1.321, 4.789, -5.1234}), Vector3d(5.4321, 7.3742, -3.465)};
  Pose3d pose2 = pose1.inverse();
  Pose3d comp1 = pose1 * pose2;
  Pose3d comp2 = pose1 * pose2;
  EXPECT_NEAR(comp1.rotation.angle(), 0.0, 1e-7);
  EXPECT_NEAR(comp1.translation.norm(), 0.0, 1e-7);
  EXPECT_NEAR(comp2.rotation.angle(), 0.0, 1e-7);
  EXPECT_NEAR(comp2.translation.norm(), 0.0, 1e-7);
}

TEST(Pose3, vector) {
  Pose3d pose1{SO3d::FromAngleAxis(1.23456, {1.321, 4.789, -5.1234}), Vector3d(5.4321, 7.3742, -3.465)};
  Pose3d pose2{SO3d::FromAngleAxis(-2.7556, {7.321, -1.789, 2.1234}), Vector3d(1.4321, -2.3742, 5.465)};
  Pose3d comp = pose1 * pose2;

  const Vector3d vec(1.23658, -6.354, 1.1561);
  const Vector3d vec1 = pose1 * (pose2 * vec);
  const Vector3d vec2 = comp * vec;

  EXPECT_NEAR(vec1.x(), vec2.x(), 1e-7);
  EXPECT_NEAR(vec1.y(), vec2.y(), 1e-7);
  EXPECT_NEAR(vec1.z(), vec2.z(), 1e-7);
}

}  // namespace isaac
