/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/math/pose2.hpp"

#include <vector>

#include "engine/core/constants.hpp"
#include "gtest/gtest.h"

namespace {
// Threshold used when testing pose equality
constexpr double kTestThreshold = 1e-6;
}

namespace isaac {

TEST(Pose2, composition) {
  Pose2d pose1{SO2d::FromAngle(Pi<double>/2), Vector2d(2.0, 0.0)};
  Pose2d pose2{SO2d::FromAngle(Pi<double>/4), Vector2d(3.0, 0.0)};
  Pose2d comp1 = pose1 * pose2;
  Pose2d comp2 = pose2 * pose1;
  EXPECT_NEAR(comp1.rotation.angle(), 3.0 * Pi<double> / 4.0, 1e-7);
  EXPECT_NEAR(comp1.translation.x(), 2.0, 1e-7);
  EXPECT_NEAR(comp1.translation.y(), 3.0, 1e-7);
  EXPECT_NEAR(comp2.rotation.angle(), 3.0 * Pi<double> / 4.0, 1e-7);
  EXPECT_NEAR(comp2.translation.x(), std::sqrt(2) + 3.0, 1e-7);
  EXPECT_NEAR(comp2.translation.y(), std::sqrt(2), 1e-7);
}

TEST(Pose2, inverse) {
  Pose2d pose1{SO2d::FromAngle(1.23456), Vector2d(5.4321, 7.3742)};
  Pose2d pose2 = pose1.inverse();
  Pose2d comp1 = pose1 * pose2;
  Pose2d comp2 = pose1 * pose2;
  EXPECT_NEAR(comp1.rotation.angle(), 0.0, 1e-7);
  EXPECT_NEAR(comp1.translation.norm(), 0.0, 1e-7);
  EXPECT_NEAR(comp2.rotation.angle(), 0.0, 1e-7);
  EXPECT_NEAR(comp2.translation.norm(), 0.0, 1e-7);
}

TEST(Pose2, vector) {
  Pose2d pose1{SO2d::FromAngle(1.23456), Vector2d(5.4321, 7.3742)};
  Pose2d pose2{SO2d::FromAngle(-2.7556), Vector2d(1.4321, -2.3742)};
  Pose2d comp = pose1 * pose2;

  const Vector2d vec(1.23658, -6.354);
  const Vector2d vec1 = pose1 * (pose2 * vec);
  const Vector2d vec2 = comp * vec;

  EXPECT_NEAR(vec1.x(), vec2.x(), 1e-7);
  EXPECT_NEAR(vec1.y(), vec2.y(), 1e-7);
}

TEST(Pose2, ExpLog) {
  // check that log(x,y) gives a tangent from x to y
  const Vector3d tangent = Pose2Log(Pose2d{SO2d::FromAngle(0.2), Vector2d{0.3,0.4}});
  EXPECT_NEAR(tangent[0], 0.3, 1e-15);
  EXPECT_NEAR(tangent[1], 0.4, 1e-15);
  EXPECT_NEAR(tangent[2], 0.2, 1e-15);
  // check that exp and log are inverse to each other
  const Vector3d tangent1{-0.4, 1.3, DegToRad(30.0)};
  const Pose2d pose1 = Pose2Exp(tangent1);
  const Vector3d tangent2 = Pose2Log(pose1);
  const Pose2d pose2 = Pose2Exp(tangent2);
  EXPECT_NEAR(tangent1[0], tangent2[0], 1e-15);
  EXPECT_NEAR(tangent1[1], tangent2[1], 1e-15);
  EXPECT_NEAR(tangent1[2], tangent2[2], 1e-15);
  EXPECT_NEAR(pose1.translation[0], pose2.translation[0], 1e-15);
  EXPECT_NEAR(pose1.translation[1], pose2.translation[1], 1e-15);
  EXPECT_NEAR(pose1.rotation.angle(), pose2.rotation.angle(), 1e-15);
  EXPECT_NEAR(tangent2[0], -0.4, 1e-15);
  EXPECT_NEAR(tangent2[1], 1.3, 1e-15);
  EXPECT_NEAR(tangent2[2], DegToRad(30.0), 1e-15);
}

TEST(Pose2, PoseMagnitude) {
  for (const double angle : {-1.0, 0.0, 0.2}) {
    for (const double x : {-10.0, 0.0, 0.7}) {
      for (const double y : {-1.0, 0.0, 100.2}) {
        for (const double delta_angle : {-0.2, 0.0, 0.3}) {
          for (const double delta_x : {-2.0, 0.0, 0.2}) {
            for (const double delta_y : {-10.0, 0.0, 10.2}) {
              const Pose2d pose1{SO2d::FromAngle(angle), Vector2d{x, y}};
              const Pose2d pose2{SO2d::FromAngle(angle + delta_angle),
                                 Vector2d{x + delta_x, y + delta_y}};

              const double delta_q = std::abs(delta_angle);
              const double delta_p = std::sqrt(delta_x * delta_x + delta_y * delta_y);

              EXPECT_TRUE(((PoseMagnitude(pose1.inverse() * pose2) -
                            Vector2d{delta_p + kTestThreshold, delta_q + kTestThreshold})
                               .array() < 0.0)
                              .all());
              EXPECT_FALSE(((PoseMagnitude(pose1.inverse() * pose2) -
                             Vector2d{delta_p - kTestThreshold, delta_q})
                                .array() < 0.0)
                               .all());
              EXPECT_FALSE(((PoseMagnitude(pose1.inverse() * pose2) -
                             Vector2d{delta_p, delta_q - kTestThreshold})
                                .array() < 0.0)
                               .all());
            }
          }
        }
      }
    }
  }
}

TEST(Pose2, IsPoseAlmostIdentity) {
  for (const double angle : {-1.0, 0.0, 0.2}) {
    for (const double x : {-10.0, 0.0, 0.7}) {
      for (const double y : {-1.0, 0.0, 100.2}) {
        const Pose2d pose1{SO2d::FromAngle(angle), Vector2d{x, y}};
        const Pose2d pose2{SO2d::FromAngle(angle), Vector2d{x, y}};
        const Pose2d pose3{SO2d::FromAngle(angle - kTestThreshold), Vector2d{x, y}};
        const Pose2d pose4{SO2d::FromAngle(angle + kTestThreshold), Vector2d{x, y}};
        const Pose2d pose5{SO2d::FromAngle(angle), Vector2d{x - kTestThreshold, y}};
        const Pose2d pose6{SO2d::FromAngle(angle), Vector2d{x + kTestThreshold, y}};
        const Pose2d pose7{SO2d::FromAngle(angle), Vector2d{x, y - kTestThreshold}};
        const Pose2d pose8{SO2d::FromAngle(angle), Vector2d{x, y + kTestThreshold}};

        EXPECT_TRUE(IsPoseAlmostIdentity(pose1.inverse() * pose2));
        EXPECT_FALSE(IsPoseAlmostIdentity(pose1.inverse() * pose3));
        EXPECT_FALSE(IsPoseAlmostIdentity(pose1.inverse() * pose4));
        EXPECT_FALSE(IsPoseAlmostIdentity(pose1.inverse() * pose5));
        EXPECT_FALSE(IsPoseAlmostIdentity(pose1.inverse() * pose6));
        EXPECT_FALSE(IsPoseAlmostIdentity(pose1.inverse() * pose7));
        EXPECT_FALSE(IsPoseAlmostIdentity(pose1.inverse() * pose8));
      }
    }
  }
}

}  // namespace isaac
