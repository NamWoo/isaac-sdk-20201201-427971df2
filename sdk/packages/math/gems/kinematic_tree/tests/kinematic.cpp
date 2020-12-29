/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/kinematic_tree/kinematic.hpp"

#include <random>

#include "gtest/gtest.h"

#include "engine/gems/math/test_utils.hpp"
#include "packages/math/gems/kinematic_tree/constant_motor.hpp"
#include "packages/math/gems/kinematic_tree/kinematic_tree.hpp"
#include "packages/math/gems/kinematic_tree/prismatic_motor.hpp"
#include "packages/math/gems/kinematic_tree/revolute_motor.hpp"

namespace isaac {
namespace kinematic_tree {

namespace {
// Create an example of manipulator with 9 joints
KinematicTree CreateNineJointChain() {
  KinematicTree model;
  auto* link = model.addLink("root", nullptr,
                             std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis1", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.0, 0.4))));
  link = model.addLink("r1", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 1.0, 0.0)));
  link = model.addLink(
      "axis2", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.1, 0.2))));
  link = model.addLink("r2", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis3", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.0, 0.2))));
  link = model.addLink("r3", link, std::make_unique<RevoluteMotor>(Vector3d(1.0, 0.0, 0.0)));
  link = model.addLink(
      "axis4", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.1, 0.0, 0.2))));
  link = model.addLink("r4", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis5", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.3, 0.0))));
  link = model.addLink("r5", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 1.0, 0.0)));
  link = model.addLink(
      "axis6", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.0, 0.3, 0.0))));
  link = model.addLink("p6", link, std::make_unique<PrismaticMotor>(Vector3d(0.2, -0.5, 0.7)));
  link = model.addLink(
      "axis7", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.3, 0.0, 0.0))));
  link = model.addLink("r7", link, std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis8", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(0.3, 1.1, 0.0))));
  link = model.addLink("p8", link, std::make_unique<PrismaticMotor>(Vector3d(0.3, 1.0, 1.2)));
  return model;
}
}  // namespace

TEST(ik, random) {
  auto manipulator = CreateNineJointChain();
  const int end_effector_id = manipulator.getNumberOfLinks() - 1;
  int failures = 0;
  for (int i = 0; i < 2000; i++) {
    const VectorXd state = VectorXd::Random(manipulator.getMotorStateDimensions());
    const auto target = ForwardKinematic(manipulator, state, end_effector_id);
    const auto ik = InverseKinematic(manipulator, target, end_effector_id, VectorXd::Zero(9));
    if (!ik) {
      failures++;
      continue;
    }
    const auto pose = ForwardKinematic(manipulator, *ik, end_effector_id).toPose3();
    ISAAC_EXPECT_POSE_NEAR(target.toPose3(), pose, 1e-3);
  }
  EXPECT_LE(failures, 10);
}

TEST(ik, simple_revolute) {
  KinematicTree model;
  auto* link = model.addLink("root", nullptr,
                             std::make_unique<RevoluteMotor>(Vector3d(0.0, 0.0, 1.0)));
  link = model.addLink(
      "axis", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(1.0, 0.0, 0.0))));
  for (double angle = 0.0; angle < TwoPi<double>; angle += Pi<double> * 0.1) {
    const auto target = ForwardKinematic(model, Vector<double, 1>(angle), 1);
    const auto ik = InverseKinematic(model, target, 1, VectorXd::Zero(1));
    EXPECT_NE(ik, std::nullopt);
    if (ik) {
      EXPECT_NEAR(0.0, DeltaAngle(angle, (*ik)[0]), 1e-5);
    }
  }
}

TEST(ik, chain_revolute) {
  KinematicTree model;
  auto* link = model.addLink("root", nullptr,
                             std::make_unique<RevoluteMotor>(Vector3d(0.7, 0.0, 1.0)));
  link = model.addLink(
      "axis1", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(1.0, 0.0, 0.0))));
  link = model.addLink("r1", link, std::make_unique<RevoluteMotor>(Vector3d(0.5, 0.7, -0.35)));
  link = model.addLink(
      "axis2", link, std::make_unique<ConstantMotor>(Pose3d::Translation(Vector3d(-1.5, 0.1, 0.2))));
  for (double angle1 = 0.0; angle1 < TwoPi<double>; angle1 += Pi<double> * 0.1) {
    for (double angle2 = 0.0; angle2 < TwoPi<double>; angle2 += Pi<double> * 0.1) {
      const auto target = ForwardKinematic(model, Vector2d(angle1, angle2), 3);
      const auto ik = InverseKinematic(model, target, 3, VectorXd::Zero(2));
      EXPECT_NE(ik, std::nullopt);
      if (ik) {
        EXPECT_NEAR(0.0, DeltaAngle(angle1, (*ik)[0]), 1e-5);
        EXPECT_NEAR(0.0, DeltaAngle(angle2, (*ik)[1]), 1e-5);
      }
    }
  }
}

TEST(ik, simple_prismatic) {
  KinematicTree model;
  model.addLink("root", nullptr, std::make_unique<PrismaticMotor>(Vector3d(0.0, 0.0, 1.0)));
  for (double distance = -5.0; distance < 5.0; distance += 0.1) {
    const auto target = ForwardKinematic(model, Vector<double, 1>(distance), 0);
    const auto ik = InverseKinematic(model, target, 0, VectorXd::Zero(1));
    EXPECT_NE(ik, std::nullopt);
    if (ik) {
      EXPECT_NEAR(distance, (*ik)[0], 1e-5);
    }
  }
}

TEST(ik, scaled_prismatic) {
  KinematicTree model;
  model.addLink("root", nullptr, std::make_unique<PrismaticMotor>(Vector3d(-2.1, 0.5, 1.3)));
  for (double distance = -5.0; distance < 5.0; distance += 0.1) {
    const auto target = ForwardKinematic(model, Vector<double, 1>(distance), 0);
    const auto ik = InverseKinematic(model, target, 0, VectorXd::Zero(1));
    EXPECT_NE(ik, std::nullopt);
    if (ik) {
      EXPECT_NEAR(distance, (*ik)[0], 1e-5);
    }
  }
}

TEST(ik, chain_prismatic) {
  KinematicTree model;
  auto* link = model.addLink("root", nullptr,
                             std::make_unique<PrismaticMotor>(Vector3d(0.7, 0.0, 1.0)));
  link = model.addLink("p1", link, std::make_unique<PrismaticMotor>(Vector3d(0.5, 0.7, -0.35)));
  for (double distance1 = -5.0; distance1 < 5.0; distance1 += Pi<double> * 0.25) {
    for (double distance2 = -5.0; distance2 < 5.0; distance2 += Pi<double> * 0.25) {
      const auto target = ForwardKinematic(model, Vector2d(distance1, distance2), 1);
      const auto ik = InverseKinematic(model, target, 1, VectorXd::Zero(2));
      EXPECT_NE(ik, std::nullopt);
      if (ik) {
        EXPECT_NEAR(distance1, (*ik)[0], 1e-5);
        EXPECT_NEAR(distance2, (*ik)[1], 1e-5);
      }
    }
  }
}

}  // namespace planner
}  // namespace isaac
