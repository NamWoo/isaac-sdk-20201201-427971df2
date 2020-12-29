/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/addition.hpp"

#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"
#include "packages/planner_cost/gems/range_constraints_cost.hpp"

namespace isaac {
namespace planner_cost {

TEST(Addition, eval) {
  Addition function;
  RangeConstraintsCost cost1, cost2;
  cost1.setGains(Vector3d(1.0, 2.0, 3.0));
  cost1.setMinValue(Vector3d::Random());
  cost1.setMaxValue(Vector3d::Random());

  cost2.setGains(Vector3d(1.0, 2.0, 3.0));
  cost2.setMinValue(Vector3d::Random());
  cost2.setMaxValue(Vector3d::Random());

  function.addCost(&cost1);
  function.addCost(&cost2);

  for (int test = 0; test < 1000; test++) {
    const Vector3d state = Vector3d::Random();
    const double eval = function.evaluate(0.0, state);
    const double expected = cost1.evaluate(0.0, state) + cost2.evaluate(0.0, state);
    EXPECT_NEAR(eval, expected, 1e-9);
  }
}

TEST(RangeConstraintsCost, gradient) {
  Addition function;
  RangeConstraintsCost cost1, cost2;
  cost1.setGains(Vector3d(1.0, 2.0, 3.0));
  cost1.setMinValue(Vector3d::Random());
  cost1.setMaxValue(Vector3d::Random());

  cost2.setGains(Vector3d(1.0, 2.0, 3.0));
  cost2.setMinValue(Vector3d::Random());
  cost2.setMaxValue(Vector3d::Random());

  function.addCost(&cost1);
  function.addCost(&cost2);

  Vector3d gradient;
  Vector3d expected;
  for (int test = 0; test < 1000; test++) {
    const Vector3d state = Vector3d::Random();
    gradient.setZero();
    expected.setZero();
    function.addGradient(0.0, state, gradient);
    cost1.addGradient(0.0, state, expected);
    cost2.addGradient(0.0, state, expected);
    ISAAC_ASSERT_VEC_NEAR(gradient, expected, 1e-9);
  }
}

TEST(RangeConstraintsCost, hessian) {
  Addition function;
  RangeConstraintsCost cost1, cost2;
  cost1.setGains(Vector3d(1.0, 2.0, 3.0));
  cost1.setMinValue(Vector3d::Random());
  cost1.setMaxValue(Vector3d::Random());

  cost2.setGains(Vector3d(1.0, 2.0, 3.0));
  cost2.setMinValue(Vector3d::Random());
  cost2.setMaxValue(Vector3d::Random());

  function.addCost(&cost1);
  function.addCost(&cost2);

  Matrix3d hessian;
  Matrix3d expected;
  for (int test = 0; test < 1000; test++) {
    const Vector3d state = Vector3d::Random();
    hessian.setZero();
    expected.setZero();
    function.addHessian(0.0, state, hessian);
    cost1.addHessian(0.0, state, expected);
    cost2.addHessian(0.0, state, expected);
    ISAAC_EXPECT_MAT_NEAR(hessian, expected, 1e-9);
  }
}

}  // namespace planner_cost
}  // namespace isaac
