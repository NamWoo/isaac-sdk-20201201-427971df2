/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/math/quaternion.hpp"

#include <random>
#include <vector>

#include "engine/gems/math/pose_utils.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Quaternion, ExpLog) {
  std::vector<Vector3d> log_coeffs{
    Vector3d::Zero(),
    Vector3d::Constant(1e-14),
    Vector3d::Zero(),
    Vector3d{0, 0, 1.047197551},
    Vector3d{0, 0.523598776, 0},
    Vector3d{0, -0.523598776, 0},
    Vector3d{0.261799388, 0, 0}
  };
  std::vector<Vector4d> exp_coeffs{
    Vector4d{1, 0, 0, 0},
    Vector4d{1, 0, 0, 0},
    Vector4d{1, 1e-14, 1e-14, 1e-14},
    Vector4d{0.5, 0, 0, 0.866025404},
    Vector4d{0.866025404, 0, 0.5, 0},
    Vector4d{0.866025404, 0, -0.5, 0},
    Vector4d{0.965925826, 0.258819045, 0, 0}
  };
  ASSERT_EQ(log_coeffs.size(), exp_coeffs.size());

  for (size_t i = 0; i < log_coeffs.size(); i++) {
    const Quaterniond q1 = QuaternionExp(log_coeffs[i]);
    ISAAC_EXPECT_VEC_NEAR(QuaternionCoefficients(q1), exp_coeffs[i], 1e-5);
    const Quaterniond q2(exp_coeffs[i][0], exp_coeffs[i][1], exp_coeffs[i][2], exp_coeffs[i][3]);
    ISAAC_EXPECT_VEC_NEAR(log_coeffs[i], QuaternionLog(q2), 1e-5);
  }
}

TEST(Quaternion, LogRng) {
  const Vector3d max = Vector3d::Constant(1.0);
  const Vector3d min = -max;
  std::mt19937 rng(142857);
  for (int i = 0; i < 20; i++) {
    const Vector3d c1 = VectorUniformDistribution(min, max, rng);
    const Quaterniond q1 = QuaternionExp(c1);
    const Vector3d c2 = QuaternionLog(q1);
    const Quaterniond q2 = QuaternionExp(c2);
    ISAAC_EXPECT_VEC_NEAR(c1, c2, 1e-5);
    ISAAC_EXPECT_VEC_NEAR(QuaternionCoefficients(q1), QuaternionCoefficients(q2), 1e-5);
  }
}

TEST(Quaternion, QuaternionExpJacobianFactors) {
  for (int i = 1; i < 30; i++) {
    const double x = std::pow(0.1, i);
    double f1, f2;
    QuaternionExpJacobianFactors(x, f1, f2);
    double a1, a2;
    QuaternionExpJacobianFactorsApproximation(x, a1, a2);
    EXPECT_NEAR(f1, a1, 1e-4);
    EXPECT_NEAR(f2, a2, 1e-4);
  }
}

TEST(Quaternion, ExpGradientId) {
  const Matrix43d g = QuaternionExpJacobian(Vector3d::Zero().eval());
  Matrix43d expected;
  expected << 0, 0, 0,
              1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  ISAAC_EXPECT_MAT_NEAR(g, expected, 1e-5);
}

TEST(Quaternion, ExpGradientAlmostId) {
  const Matrix43d g = QuaternionExpJacobian(Vector3d::Constant(1e-10).eval());
  Matrix43d expected;
  expected << 0, 0, 0,
              1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  ISAAC_EXPECT_MAT_NEAR(g, expected, 1e-5);
}

TEST(Quaternion, ExpGradient) {
  const Matrix43d g = QuaternionExpJacobian(Vector3d{-0.552827, 0.454995, -0.752926});
  Matrix43d expected;
  expected <<  0.458594,  -0.377438, 0.624585,
               0.738252 , 0.0751353, -0.124334,
               0.0751353, 0.767704 ,  0.102331,
              -0.124334 , 0.102331 ,  0.660206;
  ISAAC_EXPECT_MAT_NEAR(g, expected, 1e-5);
}

TEST(quaternion, product_matrix) {
  std::mt19937 rng(142857);
  for (int i = 0; i < 1000; i++) {
    const Vector4d coeffs_1 = VectorNormalDistribution<double, 4>(rng);
    const Quaterniond quaternion_1(coeffs_1[0], coeffs_1[1], coeffs_1[2], coeffs_1[3]);
    const Vector4d coeffs_2 = VectorNormalDistribution<double, 4>(rng);
    const Quaterniond quaternion_2(coeffs_2[0], coeffs_2[1], coeffs_2[2], coeffs_2[3]);
    const Vector4d result_a = QuaternionProductMatrixLeft(quaternion_1) * coeffs_2;
    const Vector4d result_b = QuaternionProductMatrixRight(quaternion_2) * coeffs_1;
    const Vector4d result_c = QuaternionCoefficients(quaternion_1 * quaternion_2);
    ISAAC_EXPECT_VEC_NEAR(result_a, result_b, 1e-9);
    ISAAC_EXPECT_VEC_NEAR(result_b, result_c, 1e-9);
  }
}

}  // namespace isaac
