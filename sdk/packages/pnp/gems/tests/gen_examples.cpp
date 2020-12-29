/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Generate input data for an example C++ code that demonstrates usage of PnP solvers.
// See pnp_test for more thorough tests of the PnP API.

#include "engine/core/logger.hpp"
#include "packages/pnp/gems/pnp.hpp"
#include "packages/pnp/gems/tests/simu.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <Eigen/Dense>

// Print an Eigen matrix such that it can be copy-pasted into a C++ source file.
void PrintMatrixCode(const isaac::MatrixXd& mat) {
  for (int i = 0; i < mat.rows(); i++) {
    for (int j = 0; j < mat.cols(); j++) {
      std::cout << mat(i, j);
      if (j + 1 < mat.cols()) std::cout << ", ";
    }
    if (i + 1 == mat.rows())
      std::cout << ";\n";
    else
      std::cout << ",\n";
  }
}

// Print an isaac pose for example C++ code.
std::ostream& operator<<(std::ostream& os, const isaac::Pose3d& pose) {
  std::cout << "translation = (" << pose.translation.transpose() << "), ";
  std::cout << "angle = " << pose.rotation.angle() << ", ";
  std::cout << "axis = (" << pose.rotation.axis().transpose() << ")";
  return os;
}

// Test equivalence of two poses (a and b) are similar within tolerance parameters.
void TestPoseEquivalence(const isaac::Pose3d& a, const isaac::Pose3d& b, double max_distance = 1e-9,
                         double max_angle_degrees = 1e-9) {
  // Calculate difference in orientation.
  isaac::SO3d delta_rot = a.rotation.inverse() * b.rotation;
  double angle = std::fmod(isaac::RadToDeg(std::abs(delta_rot.angle())), 360.0);
  angle = std::min(angle, 360.0 - angle);

  // Calculate difference in position.
  // Convert world origin in camera frame to camera position in world frame. This can make
  // a significant difference in case of a moving camera far away from world origin.
  isaac::Vector3d position1 = -a.rotation.matrix().transpose() * a.translation;
  isaac::Vector3d position2 = -b.rotation.matrix().transpose() * b.translation;

  ASSERT_LT((position1 - position2).norm(), max_distance);
  ASSERT_LT(angle, max_angle_degrees);
}

// Generate an example for PnP and test EPnP on the example data.
TEST(PnpTest, TestEpnpExample) {
  // Generate synthetic camera.
  const int width = 1280;
  const int height = 720;
  const double focal = 700.0;
  isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);

  // Construct isaac::Pose3d object from the generated camera pose.
  // This is the known ground-truth pose to compare to the output of pose estimation.
  isaac::Vector3d gt_angle_axis = isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix);
  isaac::Pose3d gt_pose{isaac::SO3d::FromAngleAxis(gt_angle_axis.norm(), gt_angle_axis),
                        -camera.rotation_matrix * camera.position};
  std::cout << gt_pose << std::endl;

  // Print camera intrinsics.
  double focal_u = camera.calib_matrix(0, 0);
  double focal_v = camera.calib_matrix(1, 1);
  double principal_u = camera.calib_matrix(0, 2);
  double principal_v = camera.calib_matrix(1, 2);
  std::cout << "--------------- synthetic C++ example below this line\n";
  std::cout << "focal_u = " << focal_u << ";\n";
  std::cout << "focal_v = " << focal_v << ";\n";
  std::cout << "principal_u = " << principal_u << ";\n";
  std::cout << "principal_v = " << principal_v << ";\n";

  // Generate inlier 2D-3D matches without noise.
  // 3D points are within camera FoV between near and far planes.
  const unsigned num_points = 6;
  const double near = 2.0;
  const double far = 100.0;
  isaac::Matrix3Xd points3;
  isaac::Matrix2Xd points2;
  isaac::pnp::GenerateFovPoints(num_points, camera, near, far, &points3, &points2);

  // Print generated 2D and 3D points.
  std::cout << "isaac::Matrix3Xd points3(3," << points3.cols() << ");\n";
  std::cout << "isaac::Matrix2Xd points2(2," << points2.cols() << ");\n";
  std::cout << "points3 << \n";
  PrintMatrixCode(points3);
  std::cout << "points2 << \n";
  PrintMatrixCode(points2);
  std::cout << "--------------- end of example\n";

  // Test EPnP camera pose estimation.
  isaac::Pose3d pose;
  ASSERT_EQ(isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                              principal_v, &pose),
            isaac::pnp::Status::kSuccess);

  // Test if computed pose matches the ground-truth pose.
  TestPoseEquivalence(pose, gt_pose);

  // Test reprojection error calculation for the example code.
  isaac::Matrix3d calib_matrix;
  calib_matrix << focal_u, 0, principal_u, 0, focal_v, principal_v, 0, 0, 1;
  for (int i = 0; i < points3.cols(); i++) {
    isaac::Vector3d proj =
        calib_matrix * (pose.rotation.matrix() * points3.col(i) + pose.translation);
    isaac::Vector2d point2(proj(0) / proj(2), proj(1) / proj(2));

    // Reprojection error in pixels should be near zero.
    ASSERT_LT((points2.col(i) - point2).norm(), 1e-6);
  }
}

// Generate synthetic input for the RANSAC-EPnP pose estimation example code.
// (1) Generate a camera with a random pose.
// (2) Generate a fixed number of 2D/3D point match inliers and add a fixed percentage of outliers.
// (3) Run RANSAC-EPnP and test the result.
TEST(PnpTest, TestEpnpRansacExample) {
  // Generate synthetic data.
  const int width = 1280;
  const int height = 720;
  const double focal = 700.0;
  isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);

  // Construct isaac::Pose3d object from the generated camera pose.
  // This is the known ground-truth pose to compare to the output of pose estimation.
  isaac::Vector3d gt_angle_axis = isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix);
  isaac::Pose3d gt_pose{isaac::SO3d::FromAngleAxis(gt_angle_axis.norm(), gt_angle_axis),
                        -camera.rotation_matrix * camera.position};
  std::cout << gt_pose << std::endl;

  // Print camera intrinsics.
  double focal_u = camera.calib_matrix(0, 0);
  double focal_v = camera.calib_matrix(1, 1);
  double principal_u = camera.calib_matrix(0, 2);
  double principal_v = camera.calib_matrix(1, 2);
  std::cout << "--------------- synthetic C++ example below this line\n";
  std::cout << "focal_u = " << focal_u << ";\n";
  std::cout << "focal_v = " << focal_v << ";\n";
  std::cout << "principal_u = " << principal_u << ";\n";
  std::cout << "principal_v = " << principal_v << ";\n";

  // Generate inlier 2D-3D matches without noise.
  // 3D points are within camera FoV between near and far planes.
  const unsigned num_inliers = 7;
  const double near = 2.0;
  const double far = 6.0;
  isaac::Matrix3Xd points3;
  isaac::Matrix2Xd points2;
  isaac::pnp::GenerateFovPoints(num_inliers, camera, near, far, &points3, &points2);

  // Generate outlier 2D-3D matches
  const unsigned num_outliers = 3;
  isaac::pnp::InsertOutliers(num_outliers, camera, 30.0, &points3, &points2);

  // Print generated 2D and 3D points.
  std::cout << "isaac::Matrix3Xd points3(3," << points3.cols() << ");\n";
  std::cout << "isaac::Matrix2Xd points2(2," << points2.cols() << ");\n";
  std::cout << "points3 << \n";
  PrintMatrixCode(points3);
  std::cout << "points2 << \n";
  PrintMatrixCode(points2);
  std::cout << "--------------- end of example\n";

  // Calculate the number of ransac experiments to run.
  unsigned ransac_rounds = isaac::pnp::EvaluateRansacFormula(0.99, 0.3, 6);

  // The standard RANSAC formula is very optimistic for small input sizes. An ad-hoc fix.
  if (points3.cols() < 20) ransac_rounds = std::max(ransac_rounds, 100u);

  // Compute top pose hypotheses.
  std::random_device rnd;
  const double ransac_threshold = 1.0;
  const unsigned max_top_poses = 3;
  const unsigned rand_seed = 73;
  std::vector<isaac::pnp::PoseHypothesis> top_hypotheses = isaac::pnp::ComputeCameraPoseEpnpRansac(
      points3, points2, focal_u, focal_v, principal_u, principal_v, ransac_rounds, ransac_threshold,
      max_top_poses, rand_seed);

  std::cout << top_hypotheses.size() << " pose hypotheses:\n";

  if (top_hypotheses.size()) {
    // There is a single valid hypothesis.
    ASSERT_EQ(top_hypotheses.size(), 1);

    // Print the hypothesis.
    const auto& hyp = top_hypotheses.front();
    std::cout << hyp.pose << std::endl;
    std::cout << "score=" << hyp.score;
    std::cout << ", " << hyp.inliers.size() << " inliers";
    if (hyp.inliers.size()) {
      std::cout << ":";
      for (auto index : hyp.inliers) std::cout << " " << index;
      std::cout << std::endl;
    }

    // Calculate and print reprojection errors.
    std::cout << "Reprojection error (pixels):" << std::fixed << std::setprecision(1);
    isaac::Matrix3d calib_matrix;
    calib_matrix << focal_u, 0, principal_u, 0, focal_v, principal_v, 0, 0, 1;
    for (int i = 0; i < points3.cols(); i++) {
      isaac::Vector3d proj =
          calib_matrix * (hyp.pose.rotation.matrix() * points3.col(i) + hyp.pose.translation);
      isaac::Vector2d point2(proj(0) / proj(2), proj(1) / proj(2));
      double repr_error = (points2.col(i) - point2).norm();
      std::cout << "  " << repr_error;
    }
    std::cout << std::endl;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
