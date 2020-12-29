/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/math/dual_quaternion.hpp"

#include <random>

#include "gtest/gtest.h"

#include "engine/gems/math/test_utils.hpp"
#include "engine/gems/math/pose_utils.hpp"

namespace isaac {

TEST(pose, create) {
  std::mt19937 rng(1337);
  for (int i = 0; i < 1000; i++) {
    const Pose3d pose = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD q1 = DualQuaternionD::FromPose3(pose);
    const Vector8d coeffs = q1.coeffs();
    const DualQuaternionD q2(coeffs[0], coeffs[1], coeffs[2], coeffs[3],
                             coeffs[4], coeffs[5], coeffs[6], coeffs[7]);
    const DualQuaternionD q3 = DualQuaternionD::FromCoefficients(coeffs);
    ISAAC_EXPECT_VEC_NEAR(q2.coeffs(), coeffs, 1e-9);
    ISAAC_EXPECT_VEC_NEAR(q3.coeffs(), coeffs, 1e-9);
  }
}

TEST(pose, conversion) {
  std::mt19937 rng(1337);
  for (int i = 0; i < 1000; i++) {
    const Pose3d pose = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD dual_quaternion = DualQuaternionD::FromPose3(pose);
    ISAAC_EXPECT_POSE_NEAR(dual_quaternion.toPose3(), pose, 1e-9);
  }
}

TEST(pose, composition) {
  std::mt19937 rng(42);
  for (int i = 0; i < 1000; i++) {
    Pose3d current_pose = Pose3d::Identity();
    DualQuaternionD current_dq = DualQuaternionD::Identity();
    for (int j = 0; j < 20; j++) {
      const Pose3d pose = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
      const DualQuaternionD dual_quaternion = DualQuaternionD::FromPose3(pose);
      current_pose = current_pose * pose;
      current_dq = current_dq * dual_quaternion;
    }
    ISAAC_EXPECT_POSE_NEAR(current_dq.toPose3(), current_pose, 1e-9);
  }
}

TEST(pose, inverse) {
  std::mt19937 rng(142857);
  for (int i = 0; i < 1000; i++) {
    const Pose3d pose = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD dual_quaternion = DualQuaternionD::FromPose3(pose);
    const Pose3d pose_inverse = dual_quaternion.inverse().toPose3();
    ISAAC_EXPECT_POSE_NEAR_ID(pose_inverse * pose, 1e-9);
  }
}

TEST(dual_quaternion, product_matrix) {
  std::mt19937 rng(142857);
  for (int i = 0; i < 1000; i++) {
    const Pose3d pose_1 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const Pose3d pose_2 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD dq_1 = DualQuaternionD::FromPose3(pose_1);
    const DualQuaternionD dq_2 = DualQuaternionD::FromPose3(pose_2);
    const Vector8d result_a = dq_1.productMatrixLeft() * dq_2.coeffs();
    const Vector8d result_b = dq_2.productMatrixRight() * dq_1.coeffs();
    const Vector8d result_c = (dq_1 * dq_2).coeffs();
    ISAAC_EXPECT_VEC_NEAR(result_a, result_c, 1e-9);
    ISAAC_EXPECT_VEC_NEAR(result_b, result_c, 1e-9);
  }
}

TEST(dual_quaternion, vector_multiplication_jacobian) {
  std::mt19937 rng;
  for (int i = 0; i < 100; i++) {
    Vector3d v = Vector3d::Random();
    const Pose3d pose_1 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const Pose3d pose_2 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD dq_1 = DualQuaternionD::FromPose3(pose_1);
    const DualQuaternionD dq_2 = DualQuaternionD::FromPose3(pose_2);
    const Matrix<double,8,3> result_a = dq_1.vectorMultiplicationJacobian(v);
    const Matrix<double,8,3> result_b = dq_2.vectorMultiplicationJacobian(v);
    const Matrix<double,8,3> result_c = (dq_1 + dq_2).vectorMultiplicationJacobian(v);
    EXPECT_NEAR((result_a + result_b - result_c).norm(), 0.0, 1e-6);
  }
}

TEST(dual_quaternion, vector_multiplication_pose) {
  std::mt19937 rng;
  for (int i = 0; i < 100; i++) {
    const Vector3d v = Vector3d::Random();
    const Pose3d pose_1 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const Pose3d pose_2 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD dq_1 = DualQuaternionD::FromPose3(pose_1);
    const DualQuaternionD dq_2 = DualQuaternionD::FromPose3(pose_2);
    const Vector3d result_a = dq_1 * (dq_2 * v);
    const Vector3d result_b = (dq_1 * dq_2) * v;
    ISAAC_EXPECT_VEC_NEAR(result_a, result_b, 1e-9);
  }
}

TEST(dual_quaternion, vector_multiplication_identity) {
  std::mt19937 rng;
  for (int i = 0; i < 100; i++) {
    const Vector3d v = Vector3d::Random();
    const Pose3d pose_1 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD dq_1 = DualQuaternionD::FromPose3(pose_1);
    const DualQuaternionD dq_2 = DualQuaternionD::Identity();
    const Vector3d result_a = dq_1 * (dq_2 * v);
    const Vector3d result_b = (dq_1 * dq_2) * v;
    ISAAC_EXPECT_VEC_NEAR(result_a, result_b, 1e-9);
    const Vector3d result_c = dq_2 * v;
    ISAAC_EXPECT_VEC_NEAR(result_c, v, 1e-6);
  }
}

TEST(dual_quaternion, normalize_by_rotation) {
  std::mt19937 rng;
  for (int i = 0; i < 100; i++) {
    const Vector3d v = Vector3d::Random();
    const Pose3d pose_1 = PoseNormalDistribution(Vector4d(10.0, 5.0, 20.0, 1.5), rng);
    const DualQuaternionD dq_1 = DualQuaternionD::FromPose3(pose_1);
    const Vector3d result_a = dq_1.normalizeByRotation() * v;
    const Vector3d result_b = dq_1 * v / dq_1.real().norm();
    ISAAC_EXPECT_VEC_NEAR(result_a, result_b, 1e-9);
  }
}

TEST(dual_quaternion, exp) {
  std::vector<Vector6d> v = {
    Vector6d::Zero(),
    MakeVector({0.0, 0.0, 0.0, -1.2, 0.7, 3.2}),
    MakeVector({0.0, 0.0, 1.047197551, 0.0, 0.0, 0.0})
  };
  std::vector<Vector8d> h = {
    MakeVector({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
    MakeVector({1.0, 0.0, 0.0, 0.0, 0.0, -1.2, 0.7, 3.2}),
    MakeVector({0.5, 0.0, 0.0, 0.866025404, 0.0, 0.0, 0.0, 0.0})
  };
  ASSERT_EQ(v.size(), h.size());
  for (size_t i = 0; i < v.size(); i++) {
    ISAAC_EXPECT_VEC_NEAR(DualQuaternionD::Exp(v[i]).coeffs(), h[i], 1e-5);
  }
}

TEST(dual_quaternion, exp_gradient_0) {
  const Matrix<double, 8, 6> J = DualQuaternionD::ExpJacobian(Vector6d::Zero());
  Matrix<double, 8, 6> expected;
  expected << 0, 0, 0, 0, 0, 0,
              1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0,
              0, 0, 0, 0, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1;
  ISAAC_EXPECT_MAT_NEAR(J, expected, 1e-5);
}

TEST(dual_quaternion, exp_gradient_1) {
  const Matrix<double, 8, 6> J = DualQuaternionD::ExpJacobian(Vector6d::Constant(1.0));
  Matrix<double, 8, 6> expected;
  expected << -0.56986 , -0.56986 , -0.56986 ,         0,         0,        0,
               0.326388, -0.243472, -0.243472,         0,         0,        0,
              -0.243472,  0.326388, -0.243472,         0,         0,        0,
              -0.243472, -0.243472,  0.326388,         0,         0,        0,
               0.160557,  0.160557,  0.160557, -0.56986 , -0.56986 , -0.56986,
              -0.56986 , -1.13972 ,         0, -0.160557,  0.56986 , -0.56986,
                      0, -0.56986 , -1.13972 , -0.56986 , -0.160557,  0.56986,
              -1.13972 ,         0, -0.56986 ,  0.56986 , -0.56986 , -0.160557;
  ISAAC_EXPECT_MAT_NEAR(J, expected, 1e-5);
}

TEST(dual_quaternion, exp_gradient_2) {
  const Matrix<double, 8, 6> J = DualQuaternionD::ExpJacobian(MakeVector({
    -0.1605, -0.473378, -0.634578, 0.694319, -0.846223, 0.752804
  }));
  Matrix<double, 8, 6> expected;
  expected <<  0.143606 ,  0.423549 ,  0.567781 ,         0,         0,         0,
               0.886698 , -0.0237112, -0.0317856,         0,         0,         0,
              -0.0237112,  0.824804 , -0.0937483,         0,         0,         0,
              -0.0317856, -0.0937483,  0.769065 ,         0,         0,         0,
              -0.611788 ,  0.785005 , -0.636218 ,  0.143606,  0.423549,  0.567781,
               0.144456 , -0.247506 , -0.186006 ,  0.691092, -0.567781,  0.423549,
               0.568057 , -0.311176 , -1.03837  ,  0.567781,  0.691092, -0.143606,
               0.841988 ,  0.871461 ,  0.335439 , -0.423549,  0.143606,  0.691092;
  ISAAC_EXPECT_MAT_NEAR(J, expected, 1e-5);
}

}  // namespace isaac
