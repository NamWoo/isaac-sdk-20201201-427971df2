/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/pnp/gems/tests/simu.hpp"
#include <gtest/gtest.h>
#include <iostream>

// Print all parameters of a camera.
std::ostream& operator<<(std::ostream& os, const isaac::pnp::Camera& camera) {
  os << "resolution " << camera.width << "x" << camera.height;
  os << " focal " << camera.calib_matrix(0, 0) << " " << camera.calib_matrix(1, 1);
  os << " skew " << camera.calib_matrix(0, 1);
  os << " pp " << camera.calib_matrix(0, 2) << " " << camera.calib_matrix(1, 2) << std::endl;
  os << "camera.rotation=" << std::endl << camera.rotation_matrix << std::endl;
  os << "camera.position=(" << camera.position.transpose() << ")";
  return os;
}

// Compute centroid of a 3D point cloud.
isaac::Vector3d ComputeCentroid(const isaac::Matrix3Xd& points) {
  isaac::Vector3d centroid = isaac::VectorXd::Zero(points.rows());
  for (int i = 0; i < points.cols(); i++) {
    centroid += points.col(i);
  }
  if (points.cols()) {
    centroid /= points.cols();
  }
  return centroid;
}

// Compute standard deviation of points in a 3D point cloud along a certain direction.
double ComputeStd(const isaac::Matrix3Xd& points, const isaac::Vector3d& direction,
                  const isaac::Vector3d& centroid) {
  if (points.cols() <= 1) {
    return 0.0;
  }
  double sum_squares = 0.0;
  isaac::Vector3d dir = direction.normalized();
  for (int i = 0; i < points.cols(); i++) {
    // Project vector between the centroid and the point to the direction.
    double sample = dir.dot(points.col(i) - centroid);
    // Sum of squares.
    sum_squares += sample * sample;
  }
  // Calculate standard deviation.
  return std::sqrt(sum_squares / (points.cols() - 1));
}

// Test whether a 3x3 matrix is orthonormal and has determinant +1 up to a given tolerance.
void TestRotationMatrix(const isaac::Matrix3d& rot_matrix, double tol = 1e-9) {
  EXPECT_LT((rot_matrix * rot_matrix.transpose() - isaac::Matrix3d::Identity()).norm(), tol);
  EXPECT_NEAR(rot_matrix.determinant(), 1.0, tol);
}

// Make sure image points are inside an image of resolution width-by-height pixels and that
// the variance is non-zero (requires >1 points).
void TestImagePoints(const isaac::Matrix2Xd& points, double width, double height) {
  if (points.cols() < 1) {
    return;
  }

  int failed = 0;
  float xmin, xmax, ymin, ymax;
  xmin = xmax = points(0, 0);
  ymin = ymax = points(0, 1);
  for (int i = 0; i < points.cols(); i++) {
    float x = points(0, i);
    float y = points(1, i);
    if (x < 0 || y < 0 || x > width || y > height) {
      failed++;
    }
    if (x < xmin) {
      xmin = x;
    } else if (x > xmax) {
      xmax = x;
    }
    if (y < ymin) {
      ymin = y;
    } else if (y > ymax) {
      ymax = y;
    }
  }
  ASSERT_EQ(failed, 0);

  // Make sure there is variance.
  EXPECT_GT(xmax - xmin, 0);
  EXPECT_GT(ymax - ymin, 0);
}

// Test statistics of samples drawn from an anisotropic 3D Gaussian distribution.
// There is no theoretical guarantee that this test never fails.
// We set a loose hard-threshold on estimators based on Gaussian noise.
TEST(PnpSimulationTest, GaussianAnisotropicMeanVarianceTest) {
  // Generate samples from an anisotropic Gaussian distribution.
  isaac::pnp::Gaussian3 gaussian;
  gaussian.setOrientation(isaac::pnp::RandomVector<isaac::Vector3d>(3));
  gaussian.setMean(isaac::Vector3d(50, -200, 25.89));
  gaussian.setDeviations(isaac::Vector3d(0.3, 1.2, 1.5));
  isaac::Matrix3Xd samples = gaussian.generate(10000);

  // Compute centroid and check if it is close to the prescribed mean of the Gaussian.
  // The more samples the less likely this test will fail.
  isaac::Vector3d centroid = ComputeCentroid(samples);
  EXPECT_LT((centroid - gaussian.mean()).norm(), 0.2);

  // Compute standard deviation along each axis and verify that it is close to the prescribed
  // deviation parameters of the Gaussian.
  float std_deviation_x = ComputeStd(samples, gaussian.orientation().col(0), gaussian.mean());
  float std_deviation_y = ComputeStd(samples, gaussian.orientation().col(1), gaussian.mean());
  float std_deviation_z = ComputeStd(samples, gaussian.orientation().col(2), gaussian.mean());
  const isaac::Vector3d& ref_deviations = gaussian.deviations();
  EXPECT_NEAR(std_deviation_x, ref_deviations(0), 0.05);
  EXPECT_NEAR(std_deviation_y, ref_deviations(1), 0.05);
  EXPECT_NEAR(std_deviation_z, ref_deviations(2), 0.05);
}

// Test the output of random camera generation.
TEST(PnpSimulationTest, GenerateRandomCameraTest) {
  // Generate a camera with fixed intrinsics and random pose.
  const int width = 1280;
  const int height = 720;
  const double focal = 700.0;
  isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);

  EXPECT_EQ(camera.width, width);
  EXPECT_EQ(camera.height, height);

  // Test camera calibration matrix.
  EXPECT_EQ(camera.calib_matrix(0, 0), focal);
  EXPECT_EQ(camera.calib_matrix(1, 1), focal);
  EXPECT_EQ(camera.calib_matrix(0, 1), 0);
  EXPECT_EQ(camera.calib_matrix(0, 2), 0.5 * width);
  EXPECT_EQ(camera.calib_matrix(1, 2), 0.5 * height);
  EXPECT_EQ(camera.calib_matrix(2, 0), 0);
  EXPECT_EQ(camera.calib_matrix(2, 1), 0);
  EXPECT_EQ(camera.calib_matrix(2, 2), 1);

  // Make sure that the camera rotation matrix is a valid rotation matrix.
  SCOPED_TRACE("GenerateRandomCameraTest");
  TestRotationMatrix(camera.rotation_matrix);
}

// Test 2D image points generated by GenerateImagePoints()
TEST(PnpSimulationTest, GenerateImagePointsTest) {
  // Generate points uniformly in an image.
  const int width = 1280;
  const int height = 720;
  const int num_points = 1000;
  isaac::Matrix2Xd points = isaac::pnp::GenerateImagePoints(num_points, width, height);

  // Test the number of points generated.
  ASSERT_EQ(points.cols(), num_points);

  // Test whether the generated image points are within the image and have non-zero variance.
  SCOPED_TRACE("GenerateImagePointsTest");
  TestImagePoints(points, width, height);
}

// Test GenerateFovPoints() for randomly generated cameras by checking if the generated
// (1) 2D points are inside the image,
// (2) 2D and 3D points perfectly match,
//     Note that if the last two statements hold then all 3D points lie within the camera FoV.
// (3) 3D points have a depth between the prescribed range [near,far].
TEST(PnpSimulationTest, GenerateFovPointsTest) {
  // Generate several random cameras and test with each.
  for (int i = 0; i < 100; i++) {
    // Generate a camera with fixed intrinsics and random pose.
    const int width = 1280;
    const int height = 720;
    const int focal = 700;
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);

    // Generate valid 2D-3D matches using this camera.
    // All 3D points are supposed to be within the camera FoV.
    const double near = 2.0;
    const double far = 10.0;
    const unsigned num_points = 100;
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::pnp::GenerateFovPoints(num_points, camera, near, far, &points3, &points2);

    // Test the number of generated points.
    ASSERT_EQ(points3.cols(), num_points);
    ASSERT_EQ(points2.cols(), num_points);

    // Test whether the generated image points are within the image and have non-zero variance.
    SCOPED_TRACE("GenerateFovPointsTest");
    TestImagePoints(points2, width, height);

    // All 3D points should project to their respective 2D points.
    // Force points that project outside to (Inf,Inf) and detect them.
    isaac::Matrix2Xd proj = isaac::pnp::ProjectPoints(camera, points3, true, true);
    ASSERT_EQ(proj.rows(), 2);
    ASSERT_EQ(proj.cols(), num_points);
    isaac::VectorXd proj_errors = isaac::pnp::ColwiseNorms(proj - points2);  // reprojection errors
    ASSERT_LT(proj_errors.maxCoeff(), 1e-9);

    // Test if the depth range of the generated 3D points is in [near,far] with non-zero variance.
    isaac::VectorXd depths = isaac::pnp::ComputeDepths(points3, camera);
    float zmin = depths.minCoeff();
    float zmax = depths.maxCoeff();
    ASSERT_GT(zmin, near);
    ASSERT_LT(zmax, far);
    ASSERT_GT(zmax - zmin, 0);
  }
}

// Test GenerateFovPointsPlanar() for fixed parameters and for random cameras.
// The parameters depth, angle, deviation are parameters of GeneateFovPointsPlanar().
//  depth      Depth of the generated plane along the optical axis.
//  angle      Angle between the generated plane and the image plane.
//  deviation  Deviation of the Gaussian noise added to the 3D points along their viewing ray.
//             The noise is capped so the points are moved at most 3*deviation from the plane.
// This functions tests if
// (1) the plane angle with respect to the image is as prescribed
// (2) the plane depth along the optical axis is as prescribed
// (3) all generated 3D points lie in front of the camera
// (4) all generated 3D points lie in the generated plane
// (5) all genearted 2D points lie within the image
// (6) 3D points project to their respective 2D points up to numerical precision
//     Note that if (5) and (6) both hold then all 3D points are within the camera FoV.
void TestFovPointsPlanar(double depth, double angle, double deviation) {
  // Clamp input inside the valid range.
  if (depth <= 0) {
    depth = 1.0;
  }
  if (angle < 0 || angle >= 85) {
    angle = 85;
  }
  if (deviation < 0) {
    deviation = 0;
  }

  const double cos_angle = cos(isaac::DegToRad(angle));

  // Generate several random cameras and test with each.
  for (int i = 0; i < 100; i++) {
    // Generate a camera with fixed intrinsics and random pose.
    const int width = 1280;
    const int height = 720;
    const double focal = 700.0;
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);

    // Generate valid 2D-3D matches using this camera.
    // All 3D points are supposed to be within the camera FoV and
    // on a plane of given inclination angle in front of the camera.
    const unsigned num_points = 100;
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::Vector4d plane;
    GenerateFovPointsPlanar(num_points, camera, depth, angle, deviation, &points3, &points2,
                            &plane);

    // Test the number of generated points.
    ASSERT_EQ(points3.cols(), num_points);
    ASSERT_EQ(points2.cols(), num_points);

    // Test whether the generated image points are within the image and have non-zero variance.
    SCOPED_TRACE("TestFovPointsPlanar");
    TestImagePoints(points2, width, height);

    // Direction of the optical axis in the world frame.
    isaac::Vector3d camera_dir = camera.rotation_matrix.row(2);

    // Calculate normalized parameters of the output plane (unit normal and corresponding offset).
    isaac::Vector3d normal = plane.block(0, 0, 3, 1);
    double offset = plane(3);
    offset /= normal.norm();
    normal /= normal.norm();

    // Test if angle between plane normal and camera direction is as prescribed.
    ASSERT_NEAR(std::abs(normal.dot(camera_dir)), cos_angle, 1e-2);

    // Test if the depth of plane along the optical axis is as prescribed.
    ASSERT_NEAR(-(normal.dot(camera.position) + offset) / normal.dot(camera_dir), depth, 1e-3);

    // Test if all generated 3D points lie in front of the camera.
    isaac::VectorXd depths = isaac::pnp::ComputeDepths(points3, camera);
    ASSERT_GT(depths.minCoeff(), 0);

    // Calculate distance between each generated 3D point and the generated plane.
    isaac::VectorXd plane_distance = ((normal.transpose() * points3).array() + offset).abs();

    // Test whether all generated 3D points lie on the generated plane.
    // When the deviation parameter is non-zero, the Gaussian noise added along the viewing ray
    // is capped at 3*deviation, so this is the upper limit of the point-plane distances.
    ASSERT_LT(plane_distance.maxCoeff(), 3 * deviation + 1e-2);

    // All 3D points should project to their respective 2D points.
    isaac::Matrix2Xd proj_points = isaac::pnp::ProjectPoints(camera, points3, false, false);
    isaac::VectorXd proj_errors = isaac::pnp::ColwiseNorms(proj_points - points2);
    ASSERT_LT(proj_errors.maxCoeff(), 1e-9);
  }
}

// Test GenerateFovPointsPlanar() with different sets of fixed params but with random cameras.
TEST(PnpSimulationTest, GenerateFovPointsPlanarTest) {
  // Fronto parallel plane, no deviation from the plane.
  TestFovPointsPlanar(5, 0, 0);

  // Slanted parallel plane, no deviation from the plane.
  TestFovPointsPlanar(10, 53.35, 0);

  // Extremely slanted parallel plane with small deviations from the plane.
  TestFovPointsPlanar(10, 85, 0.2);

  // Slanted plane with relatively large deviations from the plane.
  TestFovPointsPlanar(10, 30.35, 1.0);
}

// Test InsertOutliers() by generating a random camera, a set of perfect inlier 2D-3D matches and
// insert outliers in between them. In particular test whether:
// (1) the generated outlier 2D points are within the image,
// (2) the number of generated outliers is the prescribed one,
// (3) the indices of generated outliers actually point to outliers.
TEST(PnpSimulationTest, InsertOutliersTest) {
  // Generate a camera with fixed intrinsics and random pose.
  const int width = 1280;
  const int height = 720;
  const double focal = 700.0;
  isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);

  // Generate a number of perfect 2D-3D matches (inliers) for this camera.
  // The number should be large so the outlier ratio has high precision (resolution).
  const float near = 2;
  const float far = 10;
  const unsigned num_inliers = 1000;
  isaac::Matrix3Xd points3;
  isaac::Matrix2Xd points2;
  isaac::pnp::GenerateFovPoints(num_inliers, camera, near, far, &points3, &points2);

  // Generate outlier 2D-3D matches and insert them randomly into the list of inliers.
  const unsigned num_outliers = 500;  // 33% outlier ratio
  const double outlier_radius = 30.0;
  std::vector<int> outlier_indices =
      isaac::pnp::InsertOutliers(num_outliers, camera, outlier_radius, &points3, &points2);
  // Test the number of output points and returned output indices.
  ASSERT_EQ(points3.cols(), num_inliers + num_outliers);
  ASSERT_EQ(points2.cols(), num_inliers + num_outliers);
  ASSERT_EQ(outlier_indices.size(), num_outliers);

  // Test whether the generated image points are within the image and have non-zero variance.
  SCOPED_TRACE("InsertOutliersTest");
  TestImagePoints(points2, camera.width, camera.height);

  // Separate perfect inliers from outliers based on the norm of their reprojection error.
  const double max_proj_error = 1e-1;
  isaac::Matrix2Xd proj_points = isaac::pnp::ProjectPoints(camera, points3, true, true);
  isaac::VectorXd proj_errors = isaac::pnp::ColwiseNorms(proj_points - points2);
  unsigned num_realized_outliers = (proj_errors.array() > max_proj_error).count();

  // Compare prescribed and realized outlier ratios. In theory, there is 0 chance that
  // independently generated 2D and 3D points perfectly match but there is no guarantee.
  // Allow for at most 1% smaller outlier ratio than prescribed.
  const unsigned max_false_positives = std::max(5, int(0.01 * (num_inliers + num_outliers)));
  ASSERT_GE(num_outliers, num_realized_outliers);
  ASSERT_NEAR(num_realized_outliers, num_outliers, max_false_positives);

  // Verify if returned outlier indices really point to outliers (again, with 1% tolerance).
  int false_positives = 0;
  for (auto outlier_index : outlier_indices) {
    ASSERT_GE(outlier_index, 0);
    ASSERT_LT(outlier_index, points3.cols());
    if (proj_errors(outlier_index) < max_proj_error) {
      false_positives++;
    }
  }
  ASSERT_LE(false_positives, max_false_positives);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
