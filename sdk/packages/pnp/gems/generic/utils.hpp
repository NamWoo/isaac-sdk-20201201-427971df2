/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

// Header of utility functions used for PnP and for related tests or benchmarking.

#include <limits>
#include <vector>

#include "engine/core/constants.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace pnp {

// Shorthand Inf used in PnP code.
constexpr double Inf = std::numeric_limits<double>::infinity();

// Pinhole camera represented in standard decomposed form of the full projection matrix.
// Model: point2 = EuclideanFromHomogeneous( calib_matrix * rotation_matrix * (point3 - position) )
struct Camera {
  int width;
  int height;
  Matrix3d calib_matrix;
  Matrix3d rotation_matrix;
  Vector3d position;
};

// Matrix operator of the cross product.
// Returned matrix is skew-symmetric with a diagonal of zeros.
Matrix3d CrossMatrix(const Vector3d& v);

// Convert angle-axis directly to rotation matrix using the Rodrigues formula.
// The norm of the input angle-axis is the rotation angle in radians.
// Returns a rotation matrix (orthonormal matrix with a determinant of +1).
Matrix3d MatrixFromAngleAxis(const Vector3d& angle_axis);

// Compute angle-axis corresponding to a rotation matrix.
// The rotation matrix has to be orthonormal with determinant of +1 for a meaningful output.
// Returns an angle-axis with norm in the range [0,pi] radians.
Vector3d AngleAxisFromMatrix(const Matrix3d& rot_matrix);

// Modify any angle-axis vector such that the angle is within the [0,Pi] range but without modifying
// the represented rotation. This is always possible and is useful because angle-axes are
// ambiguous outside of this range. The only remaining ambiguity is at Pi.
// A rotation around any axis by angle Pi is the same as the rotation around the flipped axis by Pi.
isaac::Vector3d WrapAngleAxis(const isaac::Vector3d& angle_axis);

// Conversion of column vectors from Euclidean to homogeneous coordinates.
// DxN input matrix, (D+1)xN output matrix with last row containing all 1's
MatrixXd HomogeneousFromEuclidean(const MatrixXd& in);

// Conversion of column vectors from homogeneous to Euclidean coordinates.
// Divides each column vector by its last element then remove this element.
// DxN input matrix, (D-1)xN output matrix.
// Returns 0x0 matrix in case of an invalid input matrix with D<2.
MatrixXd EuclideanFromHomogeneous(const MatrixXd& in);

// Project 3D points to the image and optionally invalidate projections of points
// that are behind the camera or that project outside of the image frame.
// A 2xN matrix of projection coordinates is returned, where points3 is a 3xN matrix.
// A projected point is invalidated by setting all elements of the corresponding column to Inf.
Matrix2Xd ProjectPoints(const Camera& camera, const Matrix3Xd& points3, bool invalid_outside = true,
                        bool invalid_behind = true);

// Calculate L2 norm and squared L2 norm of each column of a matrix.
VectorXd ColwiseNorms(const MatrixXd& matrix);
VectorXd ColwiseSquaredNorms(const MatrixXd& matrix);

// Compute depth of points along the optical axis of a perspective camera.
// Points behind the principal plane of the camera have negative depth.
// Returns Nx1 vector of depth values given a 3xN matrix of 3D point coordinates.
VectorXd ComputeDepths(const Matrix3Xd& points, const Camera& cam);

// Compute rigid (Euclidean) transform between two 3D point clouds in the least-squares sense.
// The result is in the form: target = rotation_matrix * source + translation
// allow_flip = true means that mirroring (flip) is allowed in the transformation
//              (rotation_matrix has either determinant +1 or -1).
// allow_flip = false guarantees that the output is a rotation matrix without flip.
// This only has an effect if the point clouds do not match well or if the problem is ill-posed.
bool RegisterPointsEucl(const Matrix3Xd& source_points, const Matrix3Xd& target_points,
                        Matrix3d* rotation_matrix, Vector3d* translation, bool allow_flip = false);

// Construct matrix from selected columns of an input matrix given a list of column indices.
// Returns an empty matrix if column indices are out of range.
template <typename MatrixType, typename Container>
MatrixType SliceMatrixColumns(const MatrixType& matrix, const Container& indices) {
  // TODO: use Eigen's slicing instead if Eigen version supports it
  MatrixType out(matrix.rows(), indices.size());
  for (unsigned k = 0; k < indices.size(); k++) {
    unsigned i = indices[k];
    if (i < 0 || i >= matrix.cols()) {
      return MatrixType();
    }
    out.col(k) = matrix.col(i);
  }
  return out;
}

// Stick two input Eigen matrices together horizontally.
// The input matrices are required to have the same number of rows otherwise operator=() throws.
template <typename MatrixType>
MatrixType StackMatricesHorizontally(const MatrixType& A, const MatrixType& B) {
  MatrixType result(A.rows(), A.cols() + B.cols());
  for (int i = 0; i < A.cols(); i++) {
    result.col(i) = A.col(i);
  }
  for (int i = 0; i < B.cols(); i++) {
    result.col(i + A.cols()) = B.col(i);
  }
  return result;
}

}  // namespace pnp
}  // namespace isaac
