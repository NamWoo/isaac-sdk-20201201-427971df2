/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

// This header is private, do not include it directly, include pnp.hpp instead.

// An implementation of the EPnP algorithm (for >=6 points) based on the following publication:
// V. Lepetit, F. Moreno-Noguer, P. Fua, EPnP: An Accurate O(n) Solution to the PnP Problem,
// International Journal Of Computer Vision (IJCV), 2009
// See manuscript at http://infoscience.epfl.ch/record/160138/files/top.pdf
// TODO: extend to 4,5 points

#include <Eigen/Dense>

#include "packages/pnp/gems/generic/utils.hpp"
#include "packages/pnp/gems/pnp.hpp"  // using status codes from the public interface

namespace isaac {
namespace pnp {
namespace epnp {

// Output pose and internal results from ComputeCameraPose()
// Output pose as cam_point = rotation * world_point + translation
// where world_point is a 3D point in world coords and cam_point in camera coords.
// Note that cam_position = -rotation.transpose() * translation,
// given that cam_point = rotation * (world_point - cam_position)
struct Result {
  Matrix3d rotation;           // camera orientation matrix
  Vector3d translation;        // translation vector
  int input_dims;              // dimensionality of the input point cloud
  Matrix3Xd ctl_points_world;  // control points in world coordinates (chosen 3D basis)
  Matrix3Xd ctl_points_cam;    // control points in camera coordinates
  MatrixXd bary_coeffs;        // barycentric coordinates of the input 3D points
  MatrixXd proj_coeffs;        // coefficient matrix in the projection equations (M in Eq.7)
  MatrixXd solution_basis;     // 4 basis vectors of the solution space of ctl_points_cam
  VectorXd singular_values;    // all singular values of proj_coeffs in increasing order
  VectorXd rss_repr_errors;    // RSS reprojection error for each solution space dimension
  int solution_dims;           // optimal #dimensions of the solution space (based on repr. error)
};

// High-level function that implements the full EPnP pipeline by using the other functions below.
// The other functions are exposed to enable detailed testing of individual parts.
// Requires the 4 standard camera intrinsic parameters (assumes pre-calibrated camera) and
// at least 6 2D-3D point correspondences without gross outliers for the result to be meaningful.
// Outputs the 3D pose of the camera and some internal results in Result given:
//  focal_u,v      relative focal lengths in horizontal / vertical pixel sizes from calibration
//  principal_u,v  principal point coordinates in pixels from calibration
//  points3        input 3D points as 3xN matrix (N>=6)
//  points2        input 2D points as 2xN matrix (in corresponding order to 3D points)
pnp::Status ComputeCameraPose(double focal_u, double focal_v, double principal_u,
                              double principal_v, const Matrix3Xd& points3,
                              const Matrix2Xd& points2, epnp::Result* result);

// Compute 3D basis aligned with a 3D point cloud: origin in the centroid and axes aligned and
// scaled with the principal components.
// Returns the basis as D+1 control points in columns of a 3x(D+1) matrix: the centroid, followed
// by axis end-points along non-vanishing principal directions in decreasing order of length.
// Possible cases:
//   D=0: all input points coincide within tolerance or no valid input (unusable for EPnP)
//   D=1: all input points are along a line up to tolerance (unusable for EPnP)
//   D=2: all input points are in a plane up to tolerance (planar case, works with EPnP)
//   D=3: input points are non-planar (works with EPnP)
// In case of failure a 3x0 matrix is returned.
Matrix3Xd ChooseBasis(const Matrix3Xd& points3, double tol = 1e-3);

// Compute barycentric coordinates of N 3D points with respect to a basis defined by
// 3 or 4 control points, subject to hom(points3) = hom(ctl_points) * bary_coords,
// where hom() denotes addition of an extra row of all 1's.
//   points3       Input 3D points as a 3xN matrix.
//   ctl_points    3xC matrix containing C = 3 or 4 control points, see ChooseBasis() for details.
// Returns a CxN matrix of barycentric coefficients with column sums equal to 1.
// These barycentric coordinates can be negative or positive with no particular bound in general
// because control points from ChooseBasis() are based on variances (PCA) and individual points
// can be arbitrarily far from the population.
MatrixXd ComputeBaryCoords(const Matrix3Xd& points3, const Matrix3Xd& ctl_points);

// Find the solution space of the projection equations (Eqs. 4-7 in the paper above).
// Projection can be written in the homogeneous linear form
//                          M*x = 0    (Eq.7 in the paper),
// where x is a vector of 9 or 12 unknown 3D camera coordinates of 3 (planar case) or 4 control
// points and M is a 2Nx9 (planar case) or 2Nx12 matrix (non-planar case).
// The least 1,2,3 or 4 singular values of M may all be close to 0 and any vector x lying in
// a 1-,2-,3- or 4-D solution space will satisfy the constraints M*x ~ 0.
// Instead of arbitrarily thresholding the singular values, EPnP considers
// the first 1,2,3 or 4 columns of the 9x4 (planar case) or 12x4 output matrix
// sol_basis as the basis of the solution space.
// See SolveControlPoints() for the solution in each case (in 1,2,3,4 dimensional solution space).
//  focal_u,v      relative focal lengths in horizontal / vertical pixel sizes
//  principal_u,v  principal point coordinates in pixels
//  bary_coords    3xN (planar case) or 4xN matrix, the barycentric coordinates of N>=6 3-D points
//  points2        2D projection coordinates of the 3D points
//  proj_coeffs    Coefficient matrix of the homogenenous form of the projections (see above)
//  sol_basis      9x4 (planar case) or 12x4 matrix, 4 basis vectors considered for solution space.
//  sing_values    9 or 12 singular values of M sorted in increasing order.
//                 The first 4 singular values correspond to columns of sol_basis.
// Returns true in case of success and outputs undefined in case of failure.
bool SolveProjConstraints(double focal_u, double focal_v, double principal_u, double principal_v,
                          const MatrixXd& bary_coords, const Matrix2Xd points2,
                          MatrixXd* proj_coeffs, MatrixXd* sol_basis, VectorXd* sing_values);

// Place elements of a vector of length 3*N into a 3xN matrix column-wise.
// In EPnP, this is applied to the 3*N solution vector of 3D camera coordinates of N control points.
// If the length of the input vector is not a multiple of 3, an empty 3x0 matrix is returned.
Matrix3Xd ReshapeToMatrix3xN(const VectorXd& vec);

// Given the solution space for the camera coordinates of the control points and given all pairwise
// distances between control points to preserve, compute weights of the linear combination
// of the basis vectors (Eq.8 in the EPnP paper) such that the control points can be obtained as
//                       ReshapeToMatrix3xN(sol_basis * weights).
// The solution space can be 1,2,3 or 4 dimensional in theory.
//  sol_basis      Solution space basis vectors output by SolveProjConstraints().
//  sol_dims       Number of basis vectors in sol_basis to consider for the solution.
//                 Supported values: 1,2 for planar and 1,2,3 for non-planar case.
//  distances      All pairwise distances between control points as returned by ComputeDistances().
//                 3 distances for 3 points (planar case) and 6 distances for 4 points (non-planar).
// Returns 4 weights. All weights are zero for unsupported sol_dims values and
// only the first sol_dims weights are non-zero in supported cases (see above).
Vector4d SolveControlPoints(const MatrixXd& sol_basis, int sol_dims, const VectorXd& distances);

// Compute distances between all possible pairs (i,j) of N input points.
// Input points are in matrix columns, and i,j are column indices.
// Every j = i+1,i+2,...,N-1 are listed first for each of i = 0,1,...,N-2
// EPnP only applies this for the 3 or 4 control points.
// Order for 4 points: (0,1)(0,2)(0,3)(1,2)(1,3)(2,3)
// Order for 3 points: (0,1)(0,2)(1,2)
VectorXd ComputeDistances(const MatrixXd& points);

// Project a set of 3D points directly given in camera coordinates into the image.
// Camera intrinsics are provided in the upper triangular 3x3 camera calibration matrix.
// Returns the 2D projections in a 2xN matrix of pixel coordinates for a 3xN input point matrix.
// This function serves to validate solutions to the projection equations and,
// therefore, intentionally does not make a distinction of points behind the camera.
Matrix2Xd ProjectPoints(const Matrix3d& calib_matrix, const Matrix3Xd& points3);

}  // namespace epnp
}  // namespace pnp
}  // namespace isaac
