/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/smooth_minimum.hpp"

#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"
#include "packages/planner_cost/gems/range_constraints_cost.hpp"

namespace isaac {
namespace planner_cost {

TEST(Addition, eval) {
  SmoothMinimum function;
  RangeConstraintsCost cost1, cost2;
  cost1.setGains(Vector3d(1.0, 2.0, 3.0));
  {
    const auto vec = Vector3d::Random();
    cost1.setMinValue(vec);
    cost1.setMaxValue(vec);
  }

  cost2.setGains(Vector3d(1.0, 2.0, 3.0));
  {
    const auto vec = Vector3d::Random();
    cost2.setMinValue(vec);
    cost2.setMaxValue(vec);
  }

  function.addCost(&cost1);
  function.addCost(&cost2);
  function.addCost(&cost1);  // Add it twice to make sure once in a while the values are close
  function.setMinimumSmoothing(200.0);

  for (int test = 0; test < 1000000; test++) {
    const Vector3d state = Vector3d::Random();
    const double eval = function.evaluate(0.0, state);
    const double expected = std::min(cost1.evaluate(0.0, state), cost2.evaluate(0.0, state));
    EXPECT_NEAR(eval, expected, 1e-2);
  }
}

TEST(Addition, maximum) {
  SmoothMinimum function;
  RangeConstraintsCost cost1, cost2;
  cost1.setGains(Vector3d(1.0, 2.0, 3.0));
  {
    const auto vec = Vector3d::Random();
    cost1.setMinValue(vec);
    cost1.setMaxValue(vec);
  }

  cost2.setGains(Vector3d(1.0, 2.0, 3.0));
  {
    const auto vec = Vector3d::Random();
    cost2.setMinValue(vec);
    cost2.setMaxValue(vec);
  }

  function.addCost(&cost1);
  function.addCost(&cost2);
  function.addCost(&cost1);  // Add it twice to make sure once in a while the values are close
  function.setMinimumSmoothing(-200.0);

  for (int test = 0; test < 1000000; test++) {
    const Vector3d state = Vector3d::Random();
    const double eval = function.evaluate(0.0, state);
    const double expected = std::max(cost1.evaluate(0.0, state), cost2.evaluate(0.0, state));
    EXPECT_NEAR(eval, expected, 1e-2);
  }
}

TEST(RangeConstraintsCost, gradient) {
  SmoothMinimum function;
  RangeConstraintsCost cost1, cost2;
  cost1.setGains(Vector3d(1.0, 2.0, 3.0));
  {
    const Vector3d vec = Vector3d::Random();
    cost1.setMinValue(vec);
    cost1.setMaxValue(vec);
  }

  cost2.setGains(Vector3d(1.0, 2.0, 3.0));
  {
    const Vector3d vec = Vector3d::Random();
    cost2.setMinValue(vec);
    cost2.setMaxValue(vec);
  }

  function.addCost(&cost1);
  function.addCost(&cost2);
  function.setMinimumSmoothing(20.0);

  Vector3d gradient;
  for (int test = 0; test < 100000; test++) {
    const Vector3d state = Vector3d::Random();
    gradient.setZero();
    function.addGradient(0.0, state, gradient);

    const Vector3d dx = Vector3d::Random() * 1e-3;
    const double expected = function.evaluate(0.0, state + dx);
    const double value_dx = function.evaluate(0.0, state) + dx.dot(gradient);
    EXPECT_NEAR(value_dx, expected, 2e-3);
  }
}

TEST(RangeConstraintsCost, hessian) {
  SmoothMinimum function;
  RangeConstraintsCost cost1, cost2;
  cost1.setGains(Vector3d(1.0, 2.0, 3.0));
  {
    const Vector3d vec = Vector3d::Random();
    cost1.setMinValue(vec);
    cost1.setMaxValue(vec);
  }

  cost2.setGains(Vector3d(2.2, 1.1, 1.9));
  {
    const Vector3d vec = Vector3d::Random();
    cost2.setMinValue(vec);
    cost2.setMaxValue(vec);
  }

  function.addCost(&cost1);
  function.addCost(&cost2);
  function.setMinimumSmoothing(20.0);

  Matrix3d hessian;
  Vector3d gradient;
  for (int test = 0; test < 100000; test++) {
    const Vector3d state = Vector3d::Random();
    gradient.setZero();
    function.addGradient(0.0, state, gradient);
    hessian.setZero();
    function.addHessian(0.0, state, hessian);

    const Vector3d dx = Vector3d::Random() * 1e-3;
    const double expected = function.evaluate(0.0, state + dx);
    const double value_dx =
        function.evaluate(0.0, state) + dx.dot(gradient) + 0.5 * dx.transpose() * hessian * dx;

    EXPECT_NEAR(value_dx, expected, 2e-6);
  }
}

}  // namespace planner_cost
}  // namespace isaac
