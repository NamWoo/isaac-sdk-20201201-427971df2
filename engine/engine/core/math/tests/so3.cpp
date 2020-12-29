/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/math/so3.hpp"

#include <vector>

#include "engine/core/constants.hpp"
#include "engine/core/math/quaternion.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(SO3, composition) {
  SO3d rot1 = SO3d::FromAngleAxis(1.1, Vector3d(1.0, 1.0, 3.0));
  SO3d rot2 = SO3d::FromAngleAxis(1.7, Vector3d(1.0, 1.0, 3.0));
  SO3d rot3 = SO3d::FromAngleAxis(2.5, Vector3d(1.0, 1.0, 3.0));
  SO3d rot4 = SO3d::FromAngleAxis(0.535, Vector3d(1.0, 1.0, 3.0));
  EXPECT_NEAR((rot1 * rot2 * rot3 * rot4).angle(),
               SO3d::FromAngleAxis(rot1.angle() + rot2.angle() + rot3.angle() + rot4.angle(),
                    Vector3d(1.0, 1.0, 3.0)).angle(), 1e-7);
}

TEST(SO3, angle) {
  SO3d rot1 = SO3d::FromAngleAxis(1.1, Vector3d(1.0, 0.0, 0.0));
  SO3d rot2 = SO3d::FromAngleAxis(-1.7, Vector3d(0.0, 1.0, 0.0));
  SO3d rot3 = SO3d::FromAngleAxis(2.5, Vector3d(0.0, 0.0, 1.0));
  SO3d rot4 = SO3d::FromAngleAxis(-0.535, Vector3d(1.0, 1.0, 1.0));
  EXPECT_NEAR(rot1.angle(), 1.1, 1e-7);
  EXPECT_NEAR(rot2.angle(), 1.7, 1e-7);
  EXPECT_NEAR(rot3.angle(), 2.5, 1e-7);
  EXPECT_NEAR(rot4.angle(), 0.535, 1e-7);
}

TEST(SO3, inverse) {
  SO3d rot1 = SO3d::FromAngleAxis(1.1, Vector3d(1.0, 0.0, 0.0));
  SO3d rot2 = SO3d::FromAngleAxis(1.7, Vector3d(0.0, 1.0, 0.0));
  SO3d rot3 = SO3d::FromAngleAxis(2.5, Vector3d(0.0, 0.0, 1.0));
  SO3d rot4 = SO3d::FromAngleAxis(0.535, Vector3d(1.0, 1.0, 1.0));
  EXPECT_NEAR(rot1.inverse().angle(), rot1.angle(), 1e-7);
  EXPECT_NEAR(rot2.inverse().angle(), rot2.angle(), 1e-7);
  EXPECT_NEAR(rot3.inverse().angle(), rot3.angle(), 1e-7);
  EXPECT_NEAR(rot4.inverse().angle(), rot4.angle(), 1e-7);

  EXPECT_NEAR((rot1.inverse().axis()+rot1.axis()).norm(), 0.0, 1e-7)
      << rot1.axis() << " vs " << rot1.inverse().axis();
  EXPECT_NEAR((rot2.inverse().axis()+rot2.axis()).norm(), 0.0, 1e-7)
      << rot2.axis() << " vs " << rot2.inverse().axis();
  EXPECT_NEAR((rot3.inverse().axis()+rot3.axis()).norm(), 0.0, 1e-7)
      << rot3.axis() << " vs " << rot3.inverse().axis();
  EXPECT_NEAR((rot4.inverse().axis()+rot4.axis()).norm(), 0.0, 1e-7)
      << rot4.axis() << " vs " << rot4.inverse().axis();
}

TEST(SO3, vector) {
  Vector3d vec1 = SO3d::FromAngleAxis(Pi<double>/2, Vector3d(0.0, 0.0, 1.0)) * Vector3d(1.0, 2.0, 3.0);
  EXPECT_NEAR(vec1.x(), -2.0, 1e-7);
  EXPECT_NEAR(vec1.y(), 1.0, 1e-7);
  EXPECT_NEAR(vec1.z(), 3.0, 1e-7);
}

TEST(SO3, euler_angles) {
  enum EulerAngles {
    kRoll  = 0,
    kPitch = 1,
    kYaw   = 2
  };

  constexpr double roll  = 1.1;
  constexpr double pitch = 1.7;
  constexpr double yaw   = 2.5;

  SO3d rot1 = SO3d::FromAngleAxis(roll,  Vector3d(1.0, 0.0, 0.0));
  SO3d rot2 = SO3d::FromAngleAxis(pitch, Vector3d(0.0, 1.0, 0.0));
  SO3d rot3 = SO3d::FromAngleAxis(yaw,   Vector3d(0.0, 0.0, 1.0));
  SO3d rot4 = rot1 * rot2 *rot3;

  Vector3d rot1_euler = rot1.eulerAnglesRPY();
  Vector3d rot2_euler = rot2.eulerAnglesRPY();
  Vector3d rot3_euler = rot3.eulerAnglesRPY();
  Vector3d rot4_euler = rot4.eulerAnglesRPY();

  EXPECT_NEAR(rot1_euler[kRoll],  roll, 1e-7);
  EXPECT_NEAR(rot1_euler[kPitch], 0.0,  1e-7);
  EXPECT_NEAR(rot1_euler[kYaw],   0.0,  1e-7);

  EXPECT_NEAR(rot2_euler[kRoll],  0.0,   1e-7);
  EXPECT_NEAR(rot2_euler[kPitch], pitch, 1e-7);
  EXPECT_NEAR(rot2_euler[kYaw],   0.0,   1e-7);

  EXPECT_NEAR(rot3_euler[kRoll],  0.0,  1e-7);
  EXPECT_NEAR(rot3_euler[kPitch], 0.0,  1e-7);
  EXPECT_NEAR(rot3_euler[kYaw],   yaw,  1e-7);

  EXPECT_NEAR(rot4_euler[kRoll],  roll,  1e-7);
  EXPECT_NEAR(rot4_euler[kPitch], pitch, 1e-7);
  EXPECT_NEAR(rot4_euler[kYaw],   yaw,   1e-7);
}

TEST(SO3, euler_angles_close_zero) {
  enum EulerAngles {
    kRoll  = 0,
    kPitch = 1,
    kYaw   = 2
  };

  for (double roll = -Pi<double> * 0.25; roll < Pi<double> * 0.25; roll += Pi<double> * 0.05) {
    for (double pitch = -Pi<double> * 0.25; pitch < Pi<double> * 0.25; pitch += Pi<double> * 0.05) {
      for (double yaw = -Pi<double> * 0.25; yaw < Pi<double> * 0.25; yaw += Pi<double> * 0.05) {
        const SO3d rot1 = SO3d::FromAngleAxis(roll,  Vector3d(1.0, 0.0, 0.0));
        const SO3d rot2 = SO3d::FromAngleAxis(pitch, Vector3d(0.0, 1.0, 0.0));
        const SO3d rot3 = SO3d::FromAngleAxis(yaw,   Vector3d(0.0, 0.0, 1.0));
        const SO3d rot4 = rot1 * rot2 *rot3;
        const Vector3d rot4_euler = rot4.eulerAnglesRPY();
        EXPECT_NEAR(rot4_euler[kRoll],  roll,  1e-7);
        EXPECT_NEAR(rot4_euler[kPitch], pitch, 1e-7);
        EXPECT_NEAR(rot4_euler[kYaw],   yaw,   1e-7);
      }
    }
  }
}

TEST(SO3, euler_angles_conversion) {
  enum EulerAngles {
    kRoll  = 0,
    kPitch = 1,
    kYaw   = 2
  };

  constexpr double roll  = 1.1;
  constexpr double pitch = 1.7;
  constexpr double yaw   = 2.5;

  const SO3d so3 = SO3d::FromEulerAnglesRPY(roll, pitch, yaw);

  const Vector3d euler_angles = so3.eulerAnglesRPY();

  EXPECT_NEAR(euler_angles[kRoll],  roll,  1e-7);
  EXPECT_NEAR(euler_angles[kPitch], pitch, 1e-7);
  EXPECT_NEAR(euler_angles[kYaw],   yaw,   1e-7);
}

TEST(SO3, rotation_jacobian) {
  Vector3d v = Vector3d::Random();
  for (double roll = -Pi<double> * 0.25; roll < Pi<double> * 0.25; roll += Pi<double> * 0.05) {
    for (double pitch = -Pi<double> * 0.25; pitch < Pi<double> * 0.25; pitch += Pi<double> * 0.05) {
      for (double yaw = -Pi<double> * 0.25; yaw < Pi<double> * 0.25; yaw += Pi<double> * 0.05) {
        const SO3d rot1 = SO3d::FromAngleAxis(roll,  Vector3d(1.0, 0.0, 0.0));
        const SO3d rot2 = SO3d::FromAngleAxis(pitch, Vector3d(0.0, 1.0, 0.0));
        // const SO3d rot3 = SO3d::FromAngleAxis(yaw,   Vector3d(0.0, 0.0, 1.0));
        const SO3d rot4 = rot1 * rot2;
        Matrix<double, 3, 4> result_a = rot4.vectorRotationJacobian(v);
        Matrix<double, 3, 4> result_b = rot1.matrix() * rot2.vectorRotationJacobian(v);
        Matrix<double, 3, 4> result_c =
            (QuaternionProductMatrixLeft(rot1.quaternion())
              * result_b.transpose()).transpose();
        for (int j = 0; j < result_a.cols(); j++) {
          ISAAC_EXPECT_VEC_NEAR(result_a.col(j), result_c.col(j), 1e-3);
        }
      }
    }
  }
}

}  // namespace isaac
