/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/pnp/gems/pnp.hpp"
#include "engine/core/logger.hpp"
#include "packages/pnp/gems/tests/simu.hpp"

#include <cmath>
#include <iostream>
#include <random>
#include <sstream>

#include <gtest/gtest.h>
#include <Eigen/Dense>

// Global number of repetitions for some randomized tests.
constexpr int kRepeatTests = 100;

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

// Test EPnP pose estimation with random camera poses and 3D points on a FRONTO-PARALLEL plane.
// Noise-free 2D-3D point matches, no outliers.
TEST(PnpTest, EpnpFrontoPlanarTest) {
  SCOPED_TRACE("EpnpFrontoPlanarTest");

  // Repeat the test with different random poses and points.
  for (int i = 0; i < kRepeatTests; i++) {
    // Generate a camera with fixed intrinsics and random pose.
    const int width = 1280;
    const int height = 720;
    const double focal = 700.0;
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // Construct isaac::Pose3d object from the generated camera pose.
    // This is the known ground-truth pose to compare to the output of pose estimation.
    isaac::Vector3d gt_angle_axis = isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix);
    isaac::Pose3d gt_pose{isaac::SO3d::FromAngleAxis(gt_angle_axis.norm(), gt_angle_axis),
                          -camera.rotation_matrix * camera.position};

    // Point generation parameters.
    const double depth = 5.0;  // Plane depth in meters
    const double angle = 0.0;  // Plane inclination angle in degrees (0 is fronto-parallel).

    // Output of point generation and pose estimation.
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::Vector4d plane;
    isaac::Pose3d pose;

    // Test EPnP camera pose estimation from 6 points on a fronto-parallel plane.
    isaac::pnp::GenerateFovPointsPlanar(6, camera, depth, angle, 0, &points3, &points2, &plane);
    ASSERT_EQ(isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                                principal_v, &pose),
              isaac::pnp::Status::kSuccess);
    TestPoseEquivalence(pose, gt_pose);

    // Same test with many points.
    isaac::pnp::GenerateFovPointsPlanar(50, camera, depth, angle, 0, &points3, &points2, &plane);
    ASSERT_EQ(isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                                principal_v, &pose),
              isaac::pnp::Status::kSuccess);
    TestPoseEquivalence(pose, gt_pose);
  }
}

// Test EPnP pose estimation with random camera poses and 3D points on a SLANTED plane.
// Noise-free 2D-3D point matches, no outliers.
TEST(PnpTest, EpnpSlantedPlanarTest) {
  SCOPED_TRACE("EpnpSlantedPlanarTest");

  // Repeat the test with different random poses and points.
  for (int i = 0; i < kRepeatTests; i++) {
    // Generate a camera with fixed intrinsics and random pose.
    const int width = 1280;
    const int height = 720;
    const double focal = 700.0;
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // Construct isaac::Pose3d object from the generated camera pose.
    // This is the known ground-truth pose to compare to the output of pose estimation.
    isaac::Vector3d gt_angle_axis = isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix);
    isaac::Pose3d gt_pose{isaac::SO3d::FromAngleAxis(gt_angle_axis.norm(), gt_angle_axis),
                          -camera.rotation_matrix * camera.position};

    // Point generation parameters.
    const double depth = 10.0;  // Plane depth in meters
    const double angle = 30.0;  // Plane inclination angle in degrees (0 is fronto-parallel).

    // Output of point generation and pose estimation.
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::Vector4d plane;
    isaac::Pose3d pose;

    // Test EPnP camera pose estimation from 6 points on a slanted plane.
    GenerateFovPointsPlanar(6, camera, depth, angle, 0, &points3, &points2, &plane);
    ASSERT_EQ(isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                                principal_v, &pose),
              isaac::pnp::Status::kSuccess);
    TestPoseEquivalence(pose, gt_pose);

    // Same test with many points.
    GenerateFovPointsPlanar(50, camera, depth, angle, 0, &points3, &points2, &plane);
    ASSERT_EQ(isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                                principal_v, &pose),
              isaac::pnp::Status::kSuccess);
    TestPoseEquivalence(pose, gt_pose);
  }
}

// Test EPnP pose estimation with random camera poses and 3D points within a depth range.
// Noise-free 2D-3D point matches, no outliers.
TEST(PnpTest, EpnpNonPlanarTest) {
  SCOPED_TRACE("EpnpNonPlanarTest");

  // Repeat the test with different random poses and points.
  for (int i = 0; i < kRepeatTests; i++) {
    // Generate a camera with fixed intrinsics and random pose.
    const int width = 1280;
    const int height = 720;
    const double focal = 700.0;
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // Construct isaac::Pose3d object from the generated camera pose.
    // This is the known ground-truth pose to compare to the output of pose estimation.
    isaac::Vector3d gt_angle_axis = isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix);
    isaac::Pose3d gt_pose{isaac::SO3d::FromAngleAxis(gt_angle_axis.norm(), gt_angle_axis),
                          -camera.rotation_matrix * camera.position};

    // Point generation parameters.
    const double near = 2.0;
    const double far = 100.0;

    // Output of point generation and pose estimation.
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::Vector4d plane;
    isaac::Pose3d pose;

    // Test EPnP pose estimation from 6 points between the near and far planes.
    GenerateFovPoints(6, camera, near, far, &points3, &points2);
    ASSERT_EQ(isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                                principal_v, &pose),
              isaac::pnp::Status::kSuccess);
    TestPoseEquivalence(pose, gt_pose);

    // Same test with many points.
    GenerateFovPoints(50, camera, near, far, &points3, &points2);
    ASSERT_EQ(isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                                principal_v, &pose),
              isaac::pnp::Status::kSuccess);
    TestPoseEquivalence(pose, gt_pose);
  }
}

// Print table of the necessary number of RANSAC experiments for different outlier ratios (columns)
// and different sample sizes (in table rows).
void PrintRansacTable(double success_rate = 0.99) {
  // Outlier ratios and sample sizes to sweep over.
  std::vector<double> outlier_ratios{0.05, 0.10, 0.20, 0.25, 0.30, 0.40, 0.50};
  std::vector<int> sample_sizes{2, 3, 4, 5, 6, 7, 8};

  // Print table header.
  std::stringstream ss;
  const int cell_width = 4;
  ss << "    ";
  for (double outlier_ratio : outlier_ratios) {
    ss << std::setw(cell_width) << 100.0 * outlier_ratio << "%";
  }
  LOG_INFO("\t%s", ss.str().c_str());

  // Print table body row by row.
  for (int sample_size : sample_sizes) {
    ss.str("");
    ss.clear();
    ss << sample_size << "-pt";
    for (double outlier_ratio : outlier_ratios) {
      ss << " " << std::setw(cell_width);
      ss << isaac::pnp::EvaluateRansacFormula(success_rate, outlier_ratio, sample_size);
    }
    LOG_INFO("\t%s", ss.str().c_str());
  }
}

// Test if isaac::pnp::EvaluateRansacFormula() returns the values given in a textbook.
TEST(PnpTest, RansacFormulaTest) {
  // Print the RANSAC table for information.
  PrintRansacTable(0.99);

  // Reference values are taken from the book of Hartley & Zisserman, 3rd ed., p. 119, Table 4.3
  std::vector<double> outlier_ratio{0.05, 0.10, 0.20, 0.25, 0.30, 0.40, 0.50};
  std::vector<int> sample_size{2, 3, 4, 5, 6, 7, 8};
  Eigen::MatrixXi num_experiments(sample_size.size(), outlier_ratio.size());
  num_experiments << 2, 3, 5, 6, 7, 11, 17, 3, 4, 7, 9, 11, 19, 35, 3, 5, 9, 13, 17, 34, 72, 4, 6,
      12, 17, 26, 57, 146, 4, 7, 16, 24, 37, 97, 293, 4, 8, 20, 33, 54, 163, 588, 5, 9, 26, 44, 78,
      272, 1177;

  // Compare the output of EvaluateRansacFormula() to the reference values.
  for (size_t i = 0; i < sample_size.size(); i++) {
    for (size_t j = 0; j < outlier_ratio.size(); j++) {
      EXPECT_EQ(isaac::pnp::EvaluateRansacFormula(0.99, outlier_ratio[j], sample_size[i]),
                num_experiments(i, j));
    }
  }
}

// Test RANSAC-EPnP pose estimation on a many random synthetic datasets contaminated with outliers.
// In each each test run:
// (1) Generate a camera with a random pose.
// (2) Generate a fixed number of 2D/3D point match inliers and add a fixed percentage of outliers.
// (3) Run RANSAC-EPnP.
// (4) Test the result and collect statistics.
TEST(PnpTest, EpnpRansacTest) {
  SCOPED_TRACE("EpnpRansacTest");

  // Fixed number of inliers and outliers.
  const double outlier_ratio = 0.3;
  const unsigned num_inliers = 20;
  const unsigned num_outliers = round(outlier_ratio / (1.0 - outlier_ratio) * num_inliers);

  // RANSAC parameters.
  const unsigned ransac_rounds = 40;
  const double ransac_threshold = 1e-3;
  const unsigned max_top_poses = 5;
  unsigned rand_seed = 0;
  std::random_device rnd;

  // Collect outlier ratio statistics over many runs for reporting.
  double min_outlier_ratio = 1.0;
  double max_outlier_ratio = 0;
  // unsigned max_hypotheses_found = 0;

  // Run RANSAC on many different random datasets,
  // each with a random camera pose and 2D/3D matches.
  const unsigned num_runs = 100;
  unsigned num_poses_found = 0;  // Total number of successful pose estimations across all runs.
  for (unsigned i = 0; i < num_runs; i++) {
    // Generate a camera with fixed intrinsics and random pose.
    const int width = 1280;
    const int height = 720;
    const double focal = 700.0;
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // Construct isaac::Pose3d object from the generated camera pose.
    // This is the known ground-truth pose to compare to the output of pose estimation.
    isaac::Vector3d gt_angle_axis = isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix);
    isaac::Pose3d gt_pose{isaac::SO3d::FromAngleAxis(gt_angle_axis.norm(), gt_angle_axis),
                          -camera.rotation_matrix * camera.position};

    // Generate inlier 2D-3D matches without noise.
    // 3D points are within camera FoV between near and far planes.
    const double near = 2.0;
    const double far = 6.0;
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::pnp::GenerateFovPoints(num_inliers, camera, near, far, &points3, &points2);

    // Generate outlier 2D-3D matches
    isaac::pnp::InsertOutliers(num_outliers, camera, 30.0, &points3, &points2);

    // Calculate reprojection error for every single point match.
    isaac::VectorXd errs =
        isaac::pnp::ColwiseNorms(points2 - ProjectPoints(camera, points3, true, true));

    // Separate inliers from outliers based on reprojection error.
    // Because generated outliers have 1.0 chance to be outliers, but that is not guarantee.
    std::vector<bool> is_inlier(errs.size());
    for (unsigned k = 0; k < errs.size(); k++) {
      is_inlier[k] = (errs(k) <= ransac_threshold);
    }
    unsigned num_actual_outliers = (errs.array() > ransac_threshold).count();
    double ratio = double(num_actual_outliers) / points3.cols();
    if (ratio < min_outlier_ratio) {
      min_outlier_ratio = ratio;
    }
    if (ratio > max_outlier_ratio) {
      max_outlier_ratio = ratio;
    }

    // Run RANSAC-EPnP camera pose estimation on the current synthetic dataset.
    rand_seed = rnd();
    auto top_hypotheses = isaac::pnp::ComputeCameraPoseEpnpRansac(
        points3, points2, focal_u, focal_v, principal_u, principal_v, ransac_rounds,
        ransac_threshold, max_top_poses, rand_seed);

    // Count successful pose estimations accross runs.
    if (top_hypotheses.size()) {
      num_poses_found++;
    }

    // Test if the output list of top hypotheses does not exceed the prescribed capacity.
    ASSERT_LE(top_hypotheses.size(), max_top_poses);

    // Check each output pose hypothesis individually.
    for (const auto& hyp : top_hypotheses) {
      // There should be at least 6 inliers for any accepted pose hypothesis.
      ASSERT_GE(hyp.inliers.size(), 6);

      // Inliers are perfect (zero reprojection error),
      // so hypothesis score must be equal to the number of inliers up to numerical precision.
      EXPECT_NEAR(hyp.score, hyp.inliers.size(), 1e-3);

      // Make sure estimated pose matches the ground truth pose.
      TestPoseEquivalence(hyp.pose, gt_pose);
    }
  }

  // Print statistics.
  unsigned min_successful_runs = 0.9 * num_runs;
  LOG_INFO("Ransac: %.2f%% outliers, %d rounds", 100.0 * min_outlier_ratio, ransac_rounds);
  LOG_INFO("Ransac: no result in %d test runs, correct pose in %d of %d cases (%d required)",
           num_runs - num_poses_found, num_poses_found, num_runs, min_successful_runs);

  // Require a high percentage of test runs to succeed.
  // Do not require all test runs to succeed because RANSAC is a randomized algorithm,
  // finding the solution is not guaranteed but should has a high chance.
  EXPECT_GE(num_poses_found, min_successful_runs);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
