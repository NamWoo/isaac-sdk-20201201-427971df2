/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/optimization/gradient_descent.hpp"

#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(GradientDescent, SquaredNorm) {
  // Squared norm of a 4-vector with minimum at 0.
  auto value_f = [](const Vector4d& state) {
    return state.squaredNorm();
  };
  auto value_and_gradient_f = [](const Vector4d& state) {
    return std::pair<double, Vector4d> {state.squaredNorm(), 2.0 * state};
  };
  auto update_f = [](const Vector4d& state, const Vector4d& tangent) {
    return state + tangent;
  };

  std::vector<Vector4d> points{{
    Vector4d::Zero(),
    Vector4d(1, 2, 3, 4),
    Vector4d(1, -1, 1, -1),
    Vector4d(111, -111, 111, -111),
    Vector4d(11111, -11111, 11111, -11111),
    Vector4d(1, -10, 100, -1000)
  }};

  constexpr unsigned kMaxIterations = 100;
  constexpr double kGradientTolerance = 1e-6;

  for (const auto& p : points) {
    Vector4d state;
    optimization::GradientDescentInfo info;
    std::tie(state, info) = optimization::GradientDescent(
        p, value_f, value_and_gradient_f, update_f, {kMaxIterations, kGradientTolerance});
    ISAAC_EXPECT_VEC_NEAR(Vector4d::Zero(), state, 1e-6);
    EXPECT_LT(info.score, 1e-6);
    EXPECT_LE(info.gradient_norm, kGradientTolerance);
    EXPECT_LE(info.num_iterations, kMaxIterations);
    ASSERT_TRUE(info.converged);
  }
}

TEST(GradientDescent, Rosenbrock) {
  // The Rosenbrock function which is quite hard for gradient descent algorithms.
  auto value_f = [](const Vector2d& state) {
    const double x = state.x();
    const double y = state.y();
    const double d1 = 1.0 - y;
    const double d2 = x - y*y;
    return d1*d1 + 100.0*d2*d2;
  };
  auto value_and_gradient_f = [](const Vector2d& state) {
    const double x = state.x();
    const double y = state.y();
    const double d1 = 1.0 - y;
    const double d2 = x - y*y;
    return std::pair<double, Vector2d>{
      d1*d1 + 100.0*d2*d2,
      Vector2d(200.0*d2, -2.0*d1 - 400.0*y*d2)
    };
  };
  auto update_f = [](const Vector2d& state, const Vector2d& tangent) {
    return state + tangent;
  };

  std::vector<Vector2d> points{{
    Vector2d(1, 1),
    Vector2d::Zero(),
    Vector2d(0, 1),
    Vector2d(-1, 1),
    Vector2d(1.5, -1),
    Vector2d(2.5, 2.5),
    Vector2d(2.5, -2.5),
    Vector2d(-2.5, -2.5),
    Vector2d(-2.5, 2.5)
  }};

  constexpr unsigned kMaxIterations = 10000;
  constexpr double kGradientTolerance = 1e-4;

  for (const auto& p : points) {
    Vector2d state;
    optimization::GradientDescentInfo info;
    std::tie(state, info) = optimization::GradientDescent(
        p, value_f, value_and_gradient_f, update_f, {kMaxIterations, kGradientTolerance});
    ISAAC_EXPECT_VEC_NEAR(Vector2d(1, 1), state, 1e-3);
    EXPECT_LT(info.score, 1e-3);
    EXPECT_LE(info.gradient_norm, kGradientTolerance);
    EXPECT_LE(info.num_iterations, kMaxIterations);
    ASSERT_TRUE(info.converged);
  }
}

}  // namespace isaac
