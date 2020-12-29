/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/pnp/gems/generic/utils.hpp"

#include <random>

#include <gtest/gtest.h>
#include "engine/core/math/pose3.hpp"

// Global number of repetitions for some randomized tests.
constexpr int kRepeatTests = 1000;

// Seed for random number generation.
constexpr int kRandomSeed = 1352;

// Global random engine used in all simulations in this file.
// Same seed and deterministic results between executions.
std::mt19937& RandomEngine() {
  static std::mt19937 rand_engine{kRandomSeed};
  return rand_engine;
}

// Generate random vector with each element uniformly distributed in [min_value, max_value].
template <typename VectorType>
VectorType RandomVector(unsigned size, typename VectorType::Scalar min_value = -1.0,
                        typename VectorType::Scalar max_value = 1.0) {
  VectorType vec(size, 1);
  std::uniform_real_distribution<typename VectorType::Scalar> distribution(min_value, max_value);
  for (int i = 0; i < vec.size(); i++) {
    vec(i) = distribution(RandomEngine());
  }
  return vec;
}

// Test whether a 3x3 matrix is orthonormal and has determinant +1 up to a given tolerance.
void TestRotationMatrix(const isaac::Matrix3d& rot_matrix, double tol = 1e-9) {
  EXPECT_LT((rot_matrix * rot_matrix.transpose() - isaac::Matrix3d::Identity()).norm(), tol);
  EXPECT_NEAR(rot_matrix.determinant(), 1.0, tol);
}

// Test for random inputs if CrossMatrix() returns a skew symmetric matrix.
TEST(PnpUtilsTest, CrossSkewSymmetricTest) {
  for (int i = 0; i < kRepeatTests; i++) {
    isaac::Matrix3d matrix = isaac::pnp::CrossMatrix(RandomVector<isaac::Vector3d>(3, -100, 100));
    ASSERT_LT((matrix.transpose() + matrix).norm(), 1e-9);
  }
}

// Test for random inputs if MatrixFromAngleAxis() returns a valid rotation matrix.
TEST(PnpUtilsTest, RodriguesRotationMatrixTest) {
  for (int i = 0; i < kRepeatTests; i++) {
    TestRotationMatrix(isaac::pnp::MatrixFromAngleAxis(RandomVector<isaac::Vector3d>(3, -100, 100)),
                       1e-9);
  }
}

// Test for random input angle-axes if the angle-axis is right null vector of the rotation matrix
// returned by MatrixFromAngleAxis().
TEST(PnpUtilsTest, RodriguesNullVectorTest) {
  for (int i = 0; i < kRepeatTests; i++) {
    isaac::Vector3d angle_axis = RandomVector<isaac::Vector3d>(3, -100, 100);
    isaac::Matrix3d rot_matrix = isaac::pnp::MatrixFromAngleAxis(angle_axis);
    ASSERT_LT((rot_matrix * angle_axis - angle_axis).norm(), 1e-9);
  }
}

// Test if AngleAxisFromMatrix() inverts MatrixFromAngleAxis().
TEST(PnpUtilsTest, RodriguesInversionTest) {
  for (int i = 0; i < kRepeatTests; i++) {
    // Generate a random angle-axis.
    isaac::Vector3d true_axis = RandomVector<isaac::Vector3d>(3, -1.0, 1.0);
    std::uniform_real_distribution<double> distribution(0, isaac::Pi<double>);
    double true_angle = distribution(RandomEngine());
    true_axis *= true_angle / true_axis.norm();

    // Recompute angle-axis from rotation matrix.
    isaac::Vector3d angle_axis =
        isaac::pnp::AngleAxisFromMatrix(isaac::pnp::MatrixFromAngleAxis(true_axis));

    // Test of the resulting angle axis is the same as the original one.
    ASSERT_LT((angle_axis - true_axis).norm(), 1e-9);
    ASSERT_NEAR(angle_axis.norm(), true_angle, 1e-9);
  }
}

// Test if WrapAngleAxis() really returns an angle-axis with norm in the range [0,Pi] and if
// the rotation represented by the output and input angle-axes match.
TEST(PnpUtilsTest, WrapAngleAxisTest) {
  for (int i = 0; i < kRepeatTests; i++) {
    // Generate a random
    isaac::Vector3d angle_axis = RandomVector<isaac::Vector3d>(3, -1.0, 1.0);
    angle_axis.normalize();

    // Generate a random angle in the [0, 100*Pi] range.
    std::uniform_real_distribution<double> distribution(0, 100 * isaac::Pi<double>);
    double angle = distribution(RandomEngine());

    // Random angle-axis with an angle in a wide range.
    angle_axis *= angle;

    // Wrap angle-axis into [0,Pi] range.
    isaac::Vector3d wrapped_axis = isaac::pnp::WrapAngleAxis(angle_axis);

    // // Test if norm of the wrapped axis is within the range [0,Pi].
    ASSERT_TRUE(wrapped_axis.norm() >= 0);
    ASSERT_TRUE(wrapped_axis.norm() <= isaac::Pi<double>);

    // The rotation represented by the original and the wrapped angle-axes should be the same.
    // Test if the corresponding rotation matrices are the same.
    isaac::Matrix3d original_matrix = isaac::pnp::MatrixFromAngleAxis(angle_axis);
    isaac::Matrix3d wrapped_matrix = isaac::pnp::MatrixFromAngleAxis(wrapped_axis);
    ASSERT_LT((original_matrix - wrapped_matrix).norm(), 1e-9);
  }
}

// Test if two angle-axes (ax1 and ax2) represent the same rotations by taking into account
// all ambiguities of the angle-axis representation.
void TestSameAngleAxisRotations(const isaac::Vector3d& ax1, const isaac::Vector3d& ax2,
                                double tolerance = 1e-9) {
  // Map both angle-axes into the [0,Pi] range, which is always possible for any rotation.
  isaac::Vector3d angle_axis1 = isaac::pnp::WrapAngleAxis(ax1);
  isaac::Vector3d angle_axis2 = isaac::pnp::WrapAngleAxis(ax2);

  // If the angle is Pi, parallel axes (irrespective of flip) represent the same rotation.
  // This is the only remaining ambiguity in the angle range [0,Pi].
  if (std::abs(angle_axis1.norm() - isaac::Pi<double>) < tolerance) {
    // Test if two angle_axes are parallel up to numerical tolerance.
    EXPECT_LT(angle_axis1.cross(angle_axis2).norm(), tolerance);

    // The angle is not Pi.
    // The two rotations are only the same if the angle-axes are the same up to numerical tolerance.
  } else {
    EXPECT_LT((angle_axis1 - angle_axis2).norm(), tolerance);
  }
}

// Test consistency between isaac::SO3d and the added functions AngleAxisFromMatrix()
TEST(PnpUtilsTest, IsaacPose3ConversionsTest) {
  for (int i = 0; i < kRepeatTests; i++) {
    // Generate two random angle-axes. Special care to the angle being between 0 and Pi,
    // otherwise the references axis needs to be flipped.
    isaac::Vector3d ax1 = RandomVector<isaac::Vector3d>(3, -1, 1);
    isaac::Vector3d ax2 = RandomVector<isaac::Vector3d>(3, -1, 1);
    std::uniform_real_distribution<double> distribution(0, isaac::Pi<double>);
    ax1 = distribution(RandomEngine()) * ax1.normalized();
    ax2 = distribution(RandomEngine()) * ax2.normalized();

    // Convert angle axes to rotation matrices and test rotation matrices.
    isaac::Matrix3d rot_matrix1 = isaac::pnp::MatrixFromAngleAxis(ax1);
    isaac::Matrix3d rot_matrix2 = isaac::pnp::MatrixFromAngleAxis(ax2);

    // Convert rotation matrices back to angle axes and test if they match the original angle-axes.
    ASSERT_LT((ax1 - isaac::pnp::AngleAxisFromMatrix(rot_matrix1)).norm(), 1e-9);
    ASSERT_LT((ax2 - isaac::pnp::AngleAxisFromMatrix(rot_matrix2)).norm(), 1e-9);

    // Construct isaac::SO3 from the two original angle axes.
    isaac::SO3d rot1 = isaac::SO3d::FromAngleAxis(ax1.norm(), ax1);
    isaac::SO3d rot2 = isaac::SO3d::FromAngleAxis(ax2.norm(), ax2);

    // Compare the rotation matrix from isaac::SO3 to the one from MatrixFromAngleAxis().
    ASSERT_LT((rot1.matrix() - isaac::pnp::MatrixFromAngleAxis(ax1)).norm(), 1e-9);
    ASSERT_LT((rot2.matrix() - isaac::pnp::MatrixFromAngleAxis(ax2)).norm(), 1e-9);

    // Compare the angle and axis from isaac::SO3 to the original angle-axis.
    // Note: SO3::angle() returns an angle between 0 and 2*Pi (uses 2*atan2(y,x) with y>=0).
    // Unfortunately, the range [0,2*Pi] has a two-fold ambiguity for angle-axes.
    EXPECT_NEAR(isaac::RadToDeg(rot1.angle()), isaac::RadToDeg(ax1.norm()), 1e-9);
    EXPECT_NEAR(isaac::RadToDeg(rot2.angle()), isaac::RadToDeg(ax2.norm()), 1e-9);

    // Compare SO3::axis() to the original axis.
    // TODO: handle the two-fold ambiguity when the original angle happens to be Pi
    EXPECT_LT((rot1.axis() - ax1.normalized()).norm(), 1e-9);
    EXPECT_LT((rot2.axis() - ax2.normalized()).norm(), 1e-9);

    // Calculate angle-axis of the rotation difference to test composition.
    isaac::Vector3d diff = isaac::pnp::AngleAxisFromMatrix(rot_matrix2.transpose() * rot_matrix1);

    // Calculate rotation difference using the composition operator of isaac::SO3.
    // Note: composition with quaternions is in the same order as with rotation matrices.
    isaac::SO3d diff_quat = rot2.inverse() * rot1;

    // Make sure the two rotation differences result in the same rotation
    TestSameAngleAxisRotations(diff_quat.axis() * diff_quat.angle(), diff);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
