/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/range_constraints_cost.hpp"

#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace planner_cost {

TEST(RangeConstraintsCost, eval) {
  RangeConstraintsCost function;
  function.setGains(Vector3d(1.0, 2.0, 3.0));
  function.setMinValue(Vector3d(-1.0, -2.0, -3.0));
  function.setMaxValue(Vector3d(1.0, 2.0, 3.0));

  // Check in the middle
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(0.5, 1.0, -2.0)), 0.5 * 0.0, 1e-9);
  // Check first parameter (gain == 1)
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(2.0, 0.0, 0.0)), 0.5 * 1.0, 1e-9);
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(3.0, 0.0, 0.0)), 0.5 * 4.0, 1e-9);
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(-3.0, 0.0, 0.0)), 0.5 * 4.0, 1e-9);

  // Check second parameter (gain == 2)
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(0.0, 3.0, 0.0)), 0.5 * 2.0, 1e-9);
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(0.0, 4.0, 0.0)), 0.5 * 8.0, 1e-9);
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(0.0, -4.0, 0.0)), 0.5 * 8.0, 1e-9);

  // Check all together
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(-2.0, 3.0, 5.0)), 0.5 * 15.0, 1e-9);
  EXPECT_NEAR(function.evaluate(0.0, Vector3d(-3.0, -3.5, 4.0)), 0.5 * 11.5, 1e-9);
}

TEST(RangeConstraintsCost, gradient) {
  RangeConstraintsCost function;
  function.setGains(Vector3d(1.0, 2.0, 3.0));
  function.setMinValue(Vector3d(-1.0, -2.0, -3.0));
  function.setMaxValue(Vector3d(1.0, 2.0, 3.0));

  // Check in the middle
  Vector3d gradient;
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(0.5, 1.0, -2.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d::Zero(), 1e-9);
  }
  // Check first parameter (gain == 1)
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(2.0, 0.0, 0.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(1.0, 0.0, 0.0), 1e-9);
  }
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(3.0, 0.0, 0.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(2.0, 0.0, 0.0), 1e-9);
  }
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(-4.0, 0.0, 0.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(-3.0, 0.0, 0.0), 1e-9);
  }

  // Check second parameter (gain == 2)
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(0.0, 3.0, 0.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(0.0, 2.0, 0.0), 1e-9);
  }
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(0.0, 4.0, 0.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(0.0, 4.0, 0.0), 1e-9);
  }
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(0.0, -5.0, 0.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(0.0, -6.0, 0.0), 1e-9);
  }

  // Check all together
  {
    gradient.setZero();
    function.addGradient(0.0, Vector3d(-2.0, 3.0, 5.0), gradient);
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(-1.0, 2.0, 6.0), 1e-9);
  }
  {
    gradient.setZero();
    // Check that we can also call this function with block operation rather than matrix
    function.addGradient(0.0, Vector3d(-3.0, -3.25, 4.0), gradient.head<3>());
    ISAAC_ASSERT_VEC_NEAR(gradient, Vector3d(-2.0, -2.5, 3.0), 1e-9);
  }
}

TEST(RangeConstraintsCost, hessian) {
  RangeConstraintsCost function;
  function.setGains(Vector3d(1.0, 2.0, 3.0));
  function.setMinValue(Vector3d(-1.0, -2.0, -3.0));
  function.setMaxValue(Vector3d(1.0, 2.0, 3.0));

  // Check in the middle
  Matrix3d hessian;
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(0.5, 1.0, -2.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d::Zero().asDiagonal()), 1e-9);
  }
  // Check first parameter (gain == 1)
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(2.0, 0.0, 0.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(1.0, 0.0, 0.0).asDiagonal()), 1e-9);
  }
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(3.0, 0.0, 0.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(1.0, 0.0, 0.0).asDiagonal()), 1e-9);
  }
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(-4.0, 0.0, 0.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(1.0, 0.0, 0.0).asDiagonal()), 1e-9);
  }

  // Check second parameter (gain == 2)
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(0.0, 3.0, 0.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(0.0, 2.0, 0.0).asDiagonal()), 1e-9);
  }
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(0.0, 4.0, 0.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(0.0, 2.0, 0.0).asDiagonal()), 1e-9);
  }
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(0.0, -5.0, 0.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(0.0, 2.0, 0.0).asDiagonal()), 1e-9);
  }

  // Check all together
  {
    hessian.setZero();
    function.addHessian(0.0, Vector3d(-2.0, 3.0, 5.0), hessian);
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(1.0, 2.0, 3.0).asDiagonal()), 1e-9);
  }
  {
    hessian.setZero();
    // Check that we can also call this function with block operation rather than matrix
    function.addHessian(0.0, Vector3d(-3.0, -3.25, 4.0), hessian.block<3, 3>(0, 0));
    ISAAC_EXPECT_MAT_NEAR(hessian, Matrix3d(Vector3d(1.0, 2.0, 3.0).asDiagonal()), 1e-9);
  }
}

}  // namespace planner_cost
}  // namespace isaac
