/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// An implementation of the EPnP algorithm (for >=6 points) based on the following publication:
// V. Lepetit, F. Moreno-Noguer, P. Fua, EPnP: An Accurate O(n) Solution to the PnP Problem,
// International Journal Of Computer Vision (IJCV), 2009
// See manuscript at http://infoscience.epfl.ch/record/160138/files/top.pdf

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include <iostream>
#include <random>
#include "packages/pnp/gems/epnp/epnp.hpp"
#include "packages/pnp/gems/tests/simu.hpp"

// Global number of repetitions for some randomized tests.
constexpr int kRepeatTests = 1000;

// Test the output size of ChooseBasis() with 0,1,2,3 and 4 input points in general configuration.
TEST(EpnpTest, ChooseBasisNotEnoughPointsTest) {
  // Tolerance for collapsing dimensions inside ChooseBasis().
  const double tol = 1e-4;

  // Test ChooseBasis() with empty input.
  isaac::Matrix3Xd points;
  isaac::Matrix3Xd ctl_points = isaac::pnp::epnp::ChooseBasis(points, tol);
  EXPECT_EQ(ctl_points.cols(), 0);

  // Fixed set of 4 non-planar points for testing.
  points.resize(3, 4);
  points << 10, -10, 10, 1.5, 8, -3, 0, 100, 20, -30, 8, 2;

  // Test the output size of ChooseBasis() for the first N = 1, 2, 3, 4 points.
  for (int num_points = 1; num_points < points.cols(); num_points++) {
    ctl_points = isaac::pnp::epnp::ChooseBasis(points.block(0, 0, 3, num_points), tol);

    // Output size equals the number of input points up to 4 points in general configuration.
    // This only holds when points do not coincide and are not all collinear or coplanar.
    EXPECT_EQ(ctl_points.cols(), num_points);
  }
}

// Test ChooseBasis() with perfectly collinear input point set.
// It should only return two points along the line that define the basis (origin and axis end-point)
// to represent the original points.
TEST(EpnpTest, ChooseBasisCollinearTest) {
  const int num_points = 100;  // Number of input points to generate and feed to ChooseBasis().
  const double tol = 1e-2;     // Tolerance for collapsing dimensions inside ChooseBasis().

  // Test for many different random lines in space.
  for (int i = 0; i < kRepeatTests; i++) {
    // Generate a random line in space via a random direction and a random pivot point on the line.
    isaac::Vector3d line_dir = isaac::pnp::RandomVector<isaac::Vector3d>(3);
    line_dir.normalize();
    isaac::Vector3d line_pivot = isaac::pnp::RandomVector<isaac::Vector3d>(3, -100, 100);

    // Generate input points along the 3D line.
    isaac::VectorXd line_param = isaac::pnp::RandomVector<isaac::VectorXd>(num_points, -100, 100);
    isaac::Matrix3Xd points(3, num_points);
    for (int i = 0; i < num_points; i++) {
      points.col(i) = line_param(i) * line_dir + line_pivot;
    }

    // Feed the collinear points to ChooseBasis().
    isaac::Matrix3Xd ctl_points = isaac::pnp::epnp::ChooseBasis(points, tol);

    // ChooseBasis() should return 2 control points: origin and a single axis end-point on the line.
    EXPECT_EQ(ctl_points.cols(), 2);

    // Returned axis direction should be parallel to the line.
    EXPECT_LT((line_dir.cross(ctl_points.col(1) - ctl_points.col(0))).norm(), 1e-9);
  }
}

// Test if a 3D basis returned by ChooseBasis() and represented as 4 control points
// is aligned with a Gaussian distribution.
// The basis is computed from statistical samples and therefore is never perfectly aligned
// with the generator distribution. If sufficiently many samples were used, the basis will be
// be approximately aligned unless there are computation errors. There is no guarantee
// that the error is within a certain threshold. Loose thresholds are used here.
void CheckControlPoints(const isaac::Matrix3Xd& ctl_points, const isaac::pnp::Gaussian3& gaussian) {
  // This test is meant for the planar and non-planar case, not for other degenerate inputs.
  int dims = int(ctl_points.cols() - 1);
  ASSERT_TRUE(dims == 2 || dims == 3);

  // The first control point (the basis origin) should be an estimate for the mean.
  EXPECT_LT((ctl_points.col(0) - gaussian.mean()).norm(), 0.3);

  // Calculate vectors and distances between control points.
  isaac::Matrix3Xd basis_axes(3, dims);
  isaac::VectorXd axis_lengths(dims);
  for (int i = 0; i < dims; i++) {
    isaac::Vector3d axis = ctl_points.col(i + 1) - ctl_points.col(0);
    double axis_length = axis.norm();
    if (axis_length) {
      basis_axes.col(i) = axis / axis_length;
    } else {
      basis_axes.col(i) = axis;
    }
    axis_lengths(i) = axis_length;
  }

  // Test mutual orthonormality of basis axes defined by the calculated control points.
  EXPECT_LT((basis_axes.transpose() * basis_axes - isaac::MatrixXd::Identity(dims, dims)).norm(),
            1e-4);

  // Normalized axis directions of the 3D Gaussian are in the columns of the orientation matrix.
  // Test their mutual orthonormality.
  const isaac::Matrix3Xd& gaussian_axes = gaussian.orientation();
  EXPECT_LT((gaussian_axes.transpose() * gaussian_axes - isaac::Matrix3d::Identity()).norm(), 1e-4);

  // Calculate the pairwise similarity (scalar product, cosine of angle) between each basis axis
  // direction and each axis of the Gaussian distribution.
  // The i-th row is the distance between the i-th basis axis and each reference Gaussian axis.
  isaac::MatrixXd similarity_matrix = (basis_axes.transpose() * gaussian_axes).cwiseAbs();

  // Find the closest Gaussian axis to each basis axis.
  Eigen::VectorXi axis_ids = Eigen::VectorXi::Zero(dims);
  isaac::VectorXd cos_angles(dims);
  for (int i = 0; i < dims; i++) cos_angles(i) = similarity_matrix.row(i).maxCoeff(&axis_ids(i));

  // Test if basis axes (which are vectors between control points) are close to parallel
  // to the corresponding Gaussian axis. Use high threshold because of comparing a distribution
  // parameter to its statistical estimate.
  EXPECT_LT((cos_angles - isaac::VectorXd::Ones(dims)).norm(), 0.1);

  // Test if distances between control points are equal to the deviation along the corresponding
  // Gaussian axis.
  isaac::VectorXd deviations(dims);
  for (int i = 0; i < dims; i++) {
    deviations(i) = gaussian.deviations()(axis_ids(i));
  }
  EXPECT_LT((deviations - axis_lengths).norm(), 0.2);
}

// Test ChooseBasis() with a random planar point distribution.
// Points are drawn from a 3D Gaussian distribution flattened along one major axis.
TEST(EpnpTest, ChooseBasisPlanarTest) {
  // Draw 3D points from a 3D Gaussian distribution that is perfectly flat along one of its
  // major axes - equivalent to 2D Gaussian samples on a plane.
  isaac::pnp::Gaussian3 gaussian;
  gaussian.setOrientation(isaac::pnp::RandomVector<isaac::Vector3d>(3));
  gaussian.setMean(isaac::Vector3d(50, -200, 25.89));
  gaussian.setDeviations(isaac::Vector3d(0, 2.3, 1.7));  // squash the Gaussian along one dimension
  isaac::Matrix3Xd points = gaussian.generate(1000);

  // Feed the planar point configuration to ChooseBasis()
  isaac::Matrix3Xd ctl_points = isaac::pnp::epnp::ChooseBasis(points, 0.01);

  // Verify that only 3 control points are returned: origin and two direction vectors in the plane.
  ASSERT_EQ(ctl_points.cols(), 3);

  // Make sure the the basis defined by the control points is actually aligned with the planar
  // Gaussian blob used to generate the input points.
  CheckControlPoints(ctl_points, gaussian);
}

// Test ChooseBasis() with a non-planar point set drawn from a 3D Gaussian distribution.
TEST(EpnpTest, ChooseBasisNonPlanarTest) {
  // Draw 3D points from a generic 3D Gaussian distribution.
  isaac::pnp::Gaussian3 gaussian;
  gaussian.setOrientation(isaac::pnp::RandomVector<isaac::Vector3d>(3));
  gaussian.setMean(isaac::Vector3d(50, -200, 25.89));
  gaussian.setDeviations(isaac::Vector3d(0.3, 1.2, 1.5));
  isaac::Matrix3Xd points = gaussian.generate(1000);

  // Feed the point configuration to ChooseBasis()
  isaac::Matrix3Xd ctl_points = isaac::pnp::epnp::ChooseBasis(points, 0.001);

  // Verify that 4 control points are returned: origin and three axes.
  // Test if the axis lengths are properly aligned with the Gaussian blob.
  ASSERT_EQ(ctl_points.cols(), 4);
  CheckControlPoints(ctl_points, gaussian);
}

// Test ComputeBaryCoords() on 3D samples drawn from a Gaussian distribution.
TEST(EpnpTest, BaryCoordsNonPlanarTest) {
  // Run the test for different 3D sample sets drawn from the same 3D Gaussian.
  for (int i = 0; i < kRepeatTests; i++) {
    // Draw many points from a fixed non-isotropic and slanted 3D Gaussian distribution.
    isaac::pnp::Gaussian3 gaussian;
    gaussian.setOrientation(isaac::pnp::RandomVector<isaac::Vector3d>(3));
    gaussian.setMean(isaac::Vector3d(50, -200, 25.89));
    gaussian.setDeviations(isaac::Vector3d(0.3, 1.2, 1.5));
    isaac::Matrix3Xd points = gaussian.generate(100);

    // Compute 3D basis represented by 4 control points (non-planar case).
    isaac::Matrix3Xd ctl_points = isaac::pnp::epnp::ChooseBasis(points, 1e-3);
    ASSERT_EQ(ctl_points.cols(), 4);

    // Describe all input points by their barycentric coordinates in this basis.
    isaac::MatrixXd bary_coeffs = isaac::pnp::epnp::ComputeBaryCoords(points, ctl_points);

    // Given N input points, the output is a 4xN matrix of barycentric coefficients.
    ASSERT_EQ(bary_coeffs.rows(), 4);
    ASSERT_EQ(bary_coeffs.cols(), points.cols());

    // Test if barycentric coordinates are correct.
    // This also checks if barycentric coeffs sum to 1.0 for each point.
    isaac::Matrix4Xd hom_basis = isaac::pnp::HomogeneousFromEuclidean(ctl_points);
    isaac::Matrix4Xd hom_points = isaac::pnp::HomogeneousFromEuclidean(points);
    ASSERT_EQ(hom_basis.rows(), ctl_points.rows() + 1);
    ASSERT_EQ(hom_basis.cols(), ctl_points.cols());
    ASSERT_EQ(hom_points.rows(), points.rows() + 1);
    ASSERT_EQ(hom_points.cols(), points.cols());
    double max_residual = (hom_basis * bary_coeffs - hom_points).cwiseAbs().maxCoeff();
    EXPECT_LT(max_residual, 1e-9);
  }
}

// Test ComputeBaryCoords() for points in a plane.
// Points are drawn from a 3D Gaussian distribution flattened along one major axis.
TEST(EpnpTest, BaryCoordsPlanarTest) {
  for (int i = 0; i < kRepeatTests; i++) {
    // Draw many points from a fixed non-isotropic and slanted 3D Gaussian distribution.
    isaac::pnp::Gaussian3 gaussian;
    gaussian.setOrientation(isaac::pnp::RandomVector<isaac::Vector3d>(3));
    gaussian.setMean(isaac::Vector3d(50, -200, 25.89));
    gaussian.setDeviations(isaac::Vector3d(0, 1.2, 1.5));
    isaac::Matrix3Xd points = gaussian.generate(100);

    // Compute planar basis represented by only 3 control points.
    isaac::Matrix3Xd ctl_points = isaac::pnp::epnp::ChooseBasis(points, 1e-2);
    ASSERT_EQ(ctl_points.cols(), 3);

    // Describe all input points by their barycentric coordinates in this basis.
    isaac::MatrixXd bary_coeffs = isaac::pnp::epnp::ComputeBaryCoords(points, ctl_points);
    ASSERT_EQ(bary_coeffs.rows(), 3);
    ASSERT_EQ(bary_coeffs.cols(), points.cols());

    // Test if barycentric coordinates are correct.
    // This also checks if barycentric coeffs sum to 1.0 for each point.
    isaac::Matrix4Xd hom_basis = isaac::pnp::HomogeneousFromEuclidean(ctl_points);
    isaac::Matrix4Xd hom_points = isaac::pnp::HomogeneousFromEuclidean(points);
    double max_residual = (hom_basis * bary_coeffs - hom_points).cwiseAbs().maxCoeff();
    EXPECT_LT(max_residual, 1e-9);
  }
}

// Test SolveControlPoints() for the non-planar case.
// The solution is sought in the form x = B*w, where x is a solution vector of 12 elements
// (camera coordinates of 4 control points as a vector) that satisfy certain distance criteria.
// B is a known 12xD matrix of D basis vectors, and w is the vector of unknown weights.
// The test works as follows:
//  (1) Simulate a random (non-orthogonal) D-dimensional basis
//      as a random 12xD matrix B with normalized columns.
//  (2) Generate a ground-truth vector w randomly.
//  (3) Synthesize the ground-truth solution x as B*w.
//  (4) Calculate the true distances between parts of x.
//  (5) Run SolveControlPoints() given B and the distances to calculate the weights w.
//  (6) Compare the calculated weights to the known weights generated in step (2).
// The parameter dims is the number of dimensions of the solution space 1 <= D <= 4 to test with.
void ControlPointSolverNonPlanarTest(int dims) {
  // Make sure dims is between 1 and 4
  if (dims < 1) {
    dims = 1;
  }
  if (dims > 4) {
    dims = 4;
  }

  // Repeat the process (1-6) many times to test with different random bases and weights.
  for (int i = 0; i < kRepeatTests; i++) {
    // (1) Generate 4 random normalized basis vectors in a 12-D space.
    isaac::MatrixXd sol_basis = isaac::pnp::RandomUniformMatrix<isaac::MatrixXd>(12, 4, -1, 1);
    sol_basis.colwise().normalize();
    for (int i = 0; i < sol_basis.cols(); i++) {
      EXPECT_NEAR(sol_basis.col(i).norm(), 1.0, 1e-6);
    }

    // (2) Generate the true weights which are to be found by SolveControlPoints().
    isaac::Vector4d true_weights = isaac::pnp::RandomVector<isaac::Vector4d>(4, -100.0f, 100.0f);
    for (int i = dims; i < 4; i++) {
      true_weights(i) = 0;
    }

    // (3) Synthesize the ground truth solution as the linear combination of the basis vectors.
    // The solution is represented as a 3x4 matrix instead of a vector of 12 elements.
    isaac::Matrix3Xd true_ctl_points =
        isaac::pnp::epnp::ReshapeToMatrix3xN(sol_basis * true_weights);

    // (4) Calculate the true pairwise distances between control points.
    isaac::VectorXd distances = isaac::pnp::epnp::ComputeDistances(true_ctl_points);

    // SolveControlPoints(): Compute the weights given the solution basis and the distances.
    isaac::Vector4d weights = isaac::pnp::epnp::SolveControlPoints(sol_basis, dims, distances);

    // Make sure that weights are not all zeros.
    ASSERT_GT(weights.squaredNorm(), 0);

    // Make sure that the computed weights match their known values up to a single global sign,
    // because the simulation above does not guarantee that all z-coordinates are positive
    // for the control points (last row of true_ctl_points).
    double err = std::min((true_weights - weights).cwiseAbs().maxCoeff(),
                          (true_weights + weights).cwiseAbs().maxCoeff());
    ASSERT_LT(err, 1e-9);
  }
}

// Test SolveControlPoints() for non-planar input and assuming a 1-dimensional a solution space.
TEST(EpnpTest, ControlPointSolver1_NonPlanar) {
  SCOPED_TRACE("1 dimensional");
  ControlPointSolverNonPlanarTest(1);
}

// Test SolveControlPoints() for non-planar input and assuming a 2-dimensional a solution space.
TEST(EpnpTest, ControlPointSolver2_NonPlanar) {
  SCOPED_TRACE("2 dimensional");
  ControlPointSolverNonPlanarTest(2);
}

// Test SolveControlPoints() for non-planar input and assuming a 3-dimensional a solution space.
TEST(EpnpTest, ControlPointSolver3_NonPlanar) {
  SCOPED_TRACE("3 dimensional");
  ControlPointSolverNonPlanarTest(3);
}

// Test to make sure that each stage of the EPnP pipeline works as expected in the noise-free case.
// Check the output pose and internal calculation results of epnp::ComputeCameraPose(),
// given the perfect (noise-free) input points and camera passed to epnp::ComputeCameraPose().
//   result     Output provided by epnp::ComputeCameraPose().
//   camera     Ideal camera passed to epnp::ComputeCameraPose(). Contains the ground-truth pose.
//   points2,3  Ideal (noise-free) input 2D and 3D points passed to epnp::ComputeCameraPose().
void CheckEpnpResult(const isaac::pnp::epnp::Result& result, const isaac::pnp::Camera& camera,
                     const isaac::Matrix3Xd& points3, const isaac::Matrix2Xd& points2) {
  ASSERT_EQ(points3.cols(), points2.cols());

  // Input point cloud should be either planar (2D arrangement) or non-planar (3D arrangement).
  // This should be reflected in result.input_dims and the number of control points calculated.
  ASSERT_GE(result.input_dims, 2);
  ASSERT_LE(result.input_dims, 3);
  ASSERT_EQ(result.ctl_points_world.cols(), result.input_dims + 1);

  // Test barycentric coordinates of the input points in the calculated basis
  // defined by the control points.
  ASSERT_EQ(result.bary_coeffs.rows(), result.input_dims + 1);
  ASSERT_EQ(result.bary_coeffs.cols(), points3.cols());
  EXPECT_LT((isaac::pnp::HomogeneousFromEuclidean(result.ctl_points_world) * result.bary_coeffs -
             isaac::pnp::HomogeneousFromEuclidean(points3))
                .cwiseAbs()
                .maxCoeff(),
            1e-9);

  // Check the size of the coefficient matrix of the projection equations.
  // Check the size of the solution basis of the projection equation and the number of corresponding
  // singular values. All should be consistent with the number of dimensions.
  ASSERT_EQ(result.proj_coeffs.rows(), 2 * points3.cols());
  ASSERT_EQ(result.proj_coeffs.cols(), 3 * (result.input_dims + 1));
  ASSERT_EQ(result.solution_basis.rows(), 3 * (result.input_dims + 1));
  ASSERT_EQ(result.singular_values.size(), 3 * (result.input_dims + 1));
  ASSERT_EQ(result.solution_basis.cols(), 4);

  // Assuming no noise in the input points, the homogeneous projection equations should have an
  // infinite number of perfect solutions: all solutions parallel to the first basis vector of the
  // hypothesized solution space all saisfy the original equations.
  // Thus, the solution space is at least 1-dimensional in the noise-free case.
  // In other terms, the least singular value should be zero.
  const double tol = 1e-9;
  EXPECT_LT(result.singular_values(0), tol);

  // The 4 hypothesized basis vectors of the solution subspace are all singular vectors
  // of the coefficient matrix of the projection equations (linear homogeneous equations).
  // Theory says that the norm of the algebraic residual when substituting each singular vector
  // into the original homogeneous equation equals the corresponding singular value, so test this.
  // This also holds in the noisy case!
  EXPECT_NEAR((result.proj_coeffs * result.solution_basis.col(0)).norm(), result.singular_values(0),
              tol);
  EXPECT_NEAR((result.proj_coeffs * result.solution_basis.col(1)).norm(), result.singular_values(1),
              tol);
  EXPECT_NEAR((result.proj_coeffs * result.solution_basis.col(2)).norm(), result.singular_values(2),
              tol);
  EXPECT_NEAR((result.proj_coeffs * result.solution_basis.col(3)).norm(), result.singular_values(3),
              tol);

  // Since the algebraic error of the first basis vector (and its scalar multiples) is always zero,
  // the geometric (reprojection) error of the corresponding control points should also be zero.
  // Calculate reprojection errors are all zero up to numerical precision.
  // This test assumes ideal (noise-free) input.
  isaac::Matrix3Xd solution = isaac::pnp::epnp::ReshapeToMatrix3xN(result.solution_basis.col(0));
  isaac::VectorXd repr_errors =
      isaac::pnp::ColwiseNorms(points2 - isaac::pnp::epnp::ProjectPoints(
                                             camera.calib_matrix, solution * result.bary_coeffs));
  EXPECT_LT(repr_errors.maxCoeff(), 1e-9);

  // Calculate the ground-truth control points in the camera frame by transforming the computed
  // control points in the world frame with the ground-truth camera pose.
  isaac::Matrix3Xd true_ctl_points_cam =
      camera.rotation_matrix * (result.ctl_points_world.colwise() - camera.position);

  // Compare the computed control points to ground-truth in the camera frame.
  // They should match up to numerical precision because the input was noise-free.
  EXPECT_LT((result.ctl_points_cam - true_ctl_points_cam).norm(), 1e-9);

  // Test if the rotation matrix computed by EPnP is orthonormal.
  EXPECT_LT((result.rotation.transpose() * result.rotation - isaac::Matrix3d::Identity()).norm(),
            1e-9);

  // The pose is computed as the rigid transformation that takes the computed control points
  // in the world frame into the computed control points in the camera frame.
  // As the input was noise-free, the residual of this transformation should be zero.
  EXPECT_LT(((result.rotation * result.ctl_points_world - result.ctl_points_cam).colwise() +
             result.translation)
                .norm(),
            1e-9);

  // Compare the calculated camera orientation to the ground-truth.
  double rot_error = isaac::RadToDeg(
      isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix.transpose() * result.rotation).norm());

  // Compare the calculated camera position to the ground-truth.
  isaac::Vector3d camera_position = -result.rotation.transpose() * result.translation;
  double tran_error = (camera_position - camera.position).norm();

  // Alternative: directly compare the translation vectors instead of the camera positions.
  // isaac::Vector3d true_translation = - camera.rotation_matrix * camera.position;
  // double tran_error = (result.translation - true_translation).norm();

  // As the input was noise-free, the rotation and translation errors should be zero.
  EXPECT_LT(rot_error, 1e-9);   // degrees
  EXPECT_LT(tran_error, 1e-9);  // meters (position range in huge: +/-100 meters)
}

// Test integrated EPnP pose estimation pipeline in case of ideal non-planar input arrangement.
// Repeats the following randomized test procedure many times:
//  (1) Generate a camera with random pose but fixed intrinsics.
//  (2) Generate sufficient number of ideal (noise-free) random 2D-3D matches such that
//      the 3D points are in front of the camera between the near and far plane.
//  (3) Camera pose estimation from the synthetic input using EPnP.
//  (4) Test if computed pose equals ground-truth and if the intermediate results are consistent.
TEST(EpnpTest, FullNonPlanarTest) {
  // Camera intrinsic parameters.
  const int width = 1280;
  const int height = 720;
  const double focal = 700.0;

  // Repeat the pose estimation for different random inputs and random cameras (all planar).
  for (int i = 0; i < kRepeatTests; i++) {
    // (1) Generate a camera with random pose but fixed intrinsics.
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // (2) Generate sufficient number of ideal (noise-free) random 2D-3D matches such that
    //     the 3D points are in front of the camera between the near and far plane.
    const double near = 5.0;
    const double far = 10.0;
    const unsigned num_points = 6;
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::pnp::GenerateFovPoints(num_points, camera, near, far, &points3, &points2);

    // (3) Camera pose estimation from the synthetic noise-free input using EPnP.
    // EPnP should always succeed for such input.
    isaac::pnp::epnp::Result result;
    ASSERT_EQ(isaac::pnp::epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v,
                                                  points3, points2, &result),
              isaac::pnp::Status::kSuccess);

    // (4) Test if computed pose equals ground-truth and if the intermediate results are consistent.
    SCOPED_TRACE("non-planar");
    CheckEpnpResult(result, camera, points3, points2);
  }
}

// Test integrated EPnP pose estimation pipeline in case of ideal input on a fronto-parallel plane.
// Repeats the following randomized test procedure many times:
//  (1) Generate a camera with random pose but fixed intrinsics.
//  (2) Generate sufficient number of ideal (noise-free) random 2D-3D matches such that
//      the 3D points are on a FRONTO-PARALLEL PLANE in front of the camera.
//  (3) Camera pose estimation from the synthetic input using EPnP.
//  (4) Test if computed pose equals ground-truth and if the intermediate results are consistent.
TEST(EpnpTest, FullFrontoPlanarTest) {
  // Camera intrinsic parameters.
  const int width = 1280;
  const int height = 720;
  const double focal = 700;

  // Repeat the pose estimation for different random inputs and random cameras (all planar).
  for (int i = 0; i < kRepeatTests; i++) {
    // (1) Generate a camera with random pose but fixed intrinsics.
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // (2) Generate sufficient number of ideal (noise-free) random 2D-3D matches such that
    //     the 3D points are on a fronto-parallel plane in front of the camera.
    const double depth = 8.0;
    const double angle = 0.0;
    const unsigned num_points = 6;
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::Vector4d plane;
    isaac::pnp::GenerateFovPointsPlanar(num_points, camera, depth, angle, 0, &points3, &points2,
                                        &plane);

    // (3) Camera pose estimation from the synthetic noise-free input using EPnP.
    // EPnP should always succeed for such input.
    isaac::pnp::epnp::Result result;
    ASSERT_EQ(isaac::pnp::epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v,
                                                  points3, points2, &result),
              isaac::pnp::Status::kSuccess);

    // (4) Test if computed pose equals ground-truth and if the intermediate results are consistent.
    SCOPED_TRACE("fronto-parallel planar");
    CheckEpnpResult(result, camera, points3, points2);
  }
}

// Test integrated EPnP pose estimation pipeline in case of ideal input on a fronto-parallel plane.
// Repeats the following randomized test procedure many times:
//  (1) Generate a camera with random pose but fixed intrinsics.
//  (2) Generate sufficient number of ideal (noise-free) random 2D-3D matches such that
//      the 3D points are on a SLANTED PLANE in front of the camera.
//  (3) Camera pose estimation from the synthetic input using EPnP.
//  (4) Test if computed pose equals ground-truth and if the intermediate results are consistent.
TEST(EpnpTest, FullSlantedPlanarTest) {
  // Camera intrinsic parameters.
  const int width = 1280;
  const int height = 720;
  const double focal = 700.0;

  // Repeat the pose estimation for different random inputs and random cameras (all planar).
  for (int i = 0; i < kRepeatTests; i++) {
    // (1) Generate a camera with random pose but fixed intrinsics.
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // (2) Generate sufficient number of ideal (noise-free) random 2D-3D matches such that
    //     the 3D points are on a SLANTED PLANE in front of the camera.
    const double depth = 10.0;
    const double angle = 80.0;
    const unsigned num_points = 6;
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2;
    isaac::Vector4d plane;
    isaac::pnp::GenerateFovPointsPlanar(num_points, camera, depth, angle, 0, &points3, &points2,
                                        &plane);

    // (3) Camera pose estimation from the synthetic noise-free input using EPnP.
    // EPnP should always succeed for such input.
    isaac::pnp::epnp::Result result;
    ASSERT_EQ(isaac::pnp::epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v,
                                                  points3, points2, &result),
              isaac::pnp::Status::kSuccess);

    // (4) Test if computed pose equals ground-truth and if the intermediate results are consistent.
    SCOPED_TRACE("slanted planar");
    CheckEpnpResult(result, camera, points3, points2);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
