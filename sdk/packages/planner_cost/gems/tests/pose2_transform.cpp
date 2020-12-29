/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/pose2_transform.hpp"

#include <random>

#include "engine/gems/math/pose_utils.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"
#include "packages/planner_cost/gems/range_constraints_cost.hpp"

namespace isaac {
namespace planner_cost {

TEST(Addition, eval) {
  RangeConstraintsCost cost;
  cost.setGains(Vector3d(1.0, 2.0, 3.0));
  cost.setMinValue(Vector3d::Random());
  cost.setMaxValue(Vector3d::Random());
  Pose2Transform function;
  function.setCost(&cost);

  std::mt19937 rng(1337);
  for (int test = 0; test < 1000; test++) {
    const Pose2d transform = PoseNormalDistribution(Vector3d(5.0, 5.0, 2.5), rng);
    const Pose2d state = PoseNormalDistribution(Vector3d(5.0, 5.0, 2.5), rng);
    function.setPose(transform);
    const double eval = function.evaluate(
        0.0, Vector3d(state.translation.x(), state.translation.y(), state.rotation.angle()));
    const Pose2d tmp = state * transform;
    const double expected = cost.evaluate(
        0.0, Vector3d(tmp.translation.x(), tmp.translation.y(), tmp.rotation.angle()));
    EXPECT_NEAR(eval, expected, 1e-9);
  }
}

TEST(RangeConstraintsCost, gradient) {
  RangeConstraintsCost cost;
  cost.setGains(Vector3d(3.0, 2.0, 1.0));
  cost.setMinValue(Vector3d::Random());
  cost.setMaxValue(Vector3d::Random());
  Pose2Transform function;
  function.setCost(&cost);

  std::mt19937 rng(2442);
  Vector3d gradient;
  for (int test = 0; test < 100000; test++) {
    const Pose2d transform = PoseNormalDistribution(Vector3d(4.0, 4.0, 0.5), rng);
    const Pose2d pose = PoseNormalDistribution(Vector3d(4.0, 4.0, 0.5), rng);
    const Vector3d state(pose.translation.x(), pose.translation.y(), pose.rotation.angle());
    function.setPose(transform);
    gradient.setZero();
    function.addGradient(0.0, state, gradient);

    const Vector3d dx = Vector3d::Random() * 1e-3;
    const double expected = function.evaluate(0.0, state + dx);
    const double value_dx = function.evaluate(0.0, state) + dx.dot(gradient);
    EXPECT_NEAR(value_dx, expected, 5e-3 * expected);
  }
}

TEST(RangeConstraintsCost, hessian) {
  RangeConstraintsCost cost;
  cost.setGains(Vector3d(3.0, 2.0, 1.0));
  cost.setMinValue(Vector3d::Random());
  cost.setMaxValue(Vector3d::Random());
  Pose2Transform function;
  function.setCost(&cost);

  std::mt19937 rng(123);
  Matrix3d hessian;
  Vector3d gradient;
  for (int test = 0; test < 1000; test++) {
    const Pose2d transform = PoseNormalDistribution(Vector3d(4.0, 4.0, 0.5), rng);
    const Pose2d pose = PoseNormalDistribution(Vector3d(4.0, 4.0, 0.5), rng);
    const Vector3d state(pose.translation.x(), pose.translation.y(), pose.rotation.angle());
    function.setPose(transform);
    gradient.setZero();
    function.addGradient(0.0, state, gradient);
    hessian.setZero();
    function.addHessian(0.0, state, hessian);

    const Vector3d dx = Vector3d::Random() * 1e-3;
    const double expected = function.evaluate(0.0, state + dx);
    const double value_dx =
        function.evaluate(0.0, state) + dx.dot(gradient) + 0.5 * dx.transpose() * hessian * dx;

    EXPECT_NEAR(value_dx, expected, 5e-6 * expected);
  }
}

}  // namespace planner_cost
}  // namespace isaac
