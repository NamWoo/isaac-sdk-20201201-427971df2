/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Utility functions used inside the isaac::pnp gem and related tests or benchmarking.

#include "packages/pnp/gems/generic/utils.hpp"
#include <cmath>

namespace isaac {
namespace pnp {

// Matrix operator of the cross product operation.
Matrix3d CrossMatrix(const Vector3d& v) {
  Matrix3d cross_matrix;
  cross_matrix << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return cross_matrix;
}

// Conversion of angle-axis to rotation matrix using the Rodrigues-formula.
// Angle-axis is decomposed into angle and axis first then the Rodrigues-formula is applied.
Matrix3d MatrixFromAngleAxis(const Vector3d& angle_axis) {
  double angle = angle_axis.norm();
  Vector3d axis = angle_axis;

  // Normalize axis if norm (the angle in radians) is non-zero.
  if (angle) {
    axis *= (1.0 / angle);
  }

  // Rodrigues formula.
  double cos_angle = std::cos(angle);
  double sin_angle = std::sin(angle);
  return cos_angle * Matrix3d::Identity() + (1 - cos_angle) * axis * axis.transpose() +
         sin_angle * CrossMatrix(axis);
}

// Calculation of the angle axis from a rotation matrix.
// The input matrix must be an orthonormal matrix for a valid output.
Vector3d AngleAxisFromMatrix(const Matrix3d& rot_matrix) {
  Vector3d q{0.5 * (rot_matrix(2, 1) - rot_matrix(1, 2)),
             0.5 * (rot_matrix(0, 2) - rot_matrix(2, 0)),
             0.5 * (rot_matrix(1, 0) - rot_matrix(0, 1))};

  double sin_angle = q.norm();  // angle is between [0,pi]
  double cos_angle = 0.5 * (rot_matrix(0, 0) + rot_matrix(1, 1) + rot_matrix(2, 2) - 1.0);

  // No rotation (corresponds to rot_matrix = identity).
  if (sin_angle == 0.0) {
    return Vector3d{0, 0, 0};
  }

  // Handle numerical inaccuracies.
  if (sin_angle > 1.0) {
    sin_angle = 1.0;
  }
  if (cos_angle > 1.0) {
    cos_angle = 1.0;
  } else if (cos_angle < -1.0) {
    cos_angle = -1.0;
  }

  // calculate angle - it will be between [0,pi] since sina>=0
  double angle = std::atan2(sin_angle, cos_angle);

  // equivalent to angle*q.normalized()
  Vector3d angle_axis = angle * (1.0 / sin_angle) * q;

  return angle_axis;
}

// Modify any angle-axis such that the angle is within the [0,Pi] range but without modifying
// the represented rotation.
isaac::Vector3d WrapAngleAxis(const isaac::Vector3d& angle_axis) {
  // Wrap angle within [0,2*Pi] range.
  double angle = angle_axis.norm();
  angle -= static_cast<int>(angle / (isaac::TwoPi<double>)) * isaac::TwoPi<double>;

  // Angle is in (Pi,2*Pi] range.
  if (angle > isaac::Pi<double>) {
    // Flip axis and switch to complementary angle because rotation around any axis by an angle in
    // the [0,2*Pi] range is the same as rotation around the flipped axis by angle (2*Pi - alpha).
    angle = 2 * isaac::Pi<double> - angle;
    return (-angle_axis.normalized() * angle);

    // The (modified) angle is in the [0,Pi] range already.
  } else {
    return (angle * angle_axis.normalized());
  }
}

// Conversion of column vectors from Euclidean to homogeneous coordinates.
MatrixXd HomogeneousFromEuclidean(const MatrixXd& in) {
  MatrixXd out(in.rows() + 1, in.cols());
  if (out.rows()) {
    out.block(0, 0, in.rows(), in.cols()) = in;
    out.row(out.rows() - 1) = VectorXd::Ones(out.cols());
  }
  return out;
}

// Conversion of column vectors from homogeneous to Euclidean coordinates.
MatrixXd EuclideanFromHomogeneous(const MatrixXd& in) {
  if (in.rows() < 2) return MatrixXd();
  const int l = in.rows() - 1;
  MatrixXd out(l, in.cols());
  for (int i = 0; i < in.cols(); i++) {
    double hom = in(l, i);  // homogeneous coordinate
    if (hom == 0) {
      out.col(i) = VectorXd::Constant(l, Inf);
    } else {
      out.col(i) = in.block(0, i, l, 1) / hom;
    }
  }
  return out;
}

Matrix2Xd ProjectPoints(const Camera& camera, const Matrix3Xd& points3, bool invalid_outside,
                        bool invalid_behind) {
  // Allocate output matrix
  Matrix2Xd proj(2, points3.cols());

  // Precompute constant 3x3 matrix product
  Matrix3d M = camera.calib_matrix * camera.rotation_matrix;

  double width = camera.width;
  double height = camera.height;

  // Point projection and check validity
  for (int i = 0; i < points3.cols(); i++) {
    Vector3d p = M * (points3.col(i) - camera.position);
    double x, y;

    // p(2) is the depth from the principal plane (i.e. measured along the optical axis).
    // Point is at infinity or almost.
    if (std::abs(p(2)) < 1e-6) {
      x = y = Inf;

      // Point is behind the camera and should be invalidated.
    } else if (p(2) < 0 && invalid_behind) {
      x = y = Inf;

    } else {
      x = p(0) / p(2);
      y = p(1) / p(2);

      // Point is outside of the image frame and should be invalidated.
      if (invalid_outside && (x < 0.0 || y < 0.0 || x > width || y > height)) {
        x = y = Inf;
      }
    }

    proj(0, i) = x;
    proj(1, i) = y;
  }
  return proj;
}

// Calculate L2 norm of each column of a matrix.
VectorXd ColwiseNorms(const MatrixXd& matrix) {
  VectorXd norms(matrix.cols());
  for (int i = 0; i < matrix.cols(); i++) {
    norms(i) = matrix.col(i).norm();
  }
  return norms;
}

// Calculate squared L2 norm of each column of a matrix.
VectorXd ColwiseSquaredNorms(const MatrixXd& matrix) {
  VectorXd norms(matrix.cols());
  for (int i = 0; i < matrix.cols(); i++) {
    norms(i) = matrix.col(i).squaredNorm();
  }
  return norms;
}

// Compute depth of points along the optical axis of a perspective camera.
VectorXd ComputeDepths(const Matrix3Xd& points, const Camera& camera) {
  VectorXd depths(points.cols());
  for (int i = 0; i < points.cols(); i++) {
    depths(i) = camera.rotation_matrix.row(2).dot(points.col(i) - camera.position);
  }
  return depths;
}

// Compute rigid-body transform between two 3D point clouds by minimizing the sum of squared
// distances between corresponding 3D points.
// This implementation is non-iterative, based on SVD.
bool RegisterPointsEucl(const Matrix3Xd& source_points, const Matrix3Xd& target_points,
                        Matrix3d* rotation_matrix, Vector3d* translation, bool allow_flip) {
  if (rotation_matrix == nullptr || translation == nullptr) {
    return false;
  }

  if (source_points.cols() < 3) {
    return false;
  }
  if (target_points.cols() < 3) {
    return false;
  }
  if (source_points.cols() != target_points.cols()) {
    return false;
  }
  const int num_points = source_points.cols();

  // Calculate the centroids of the source and target point clouds.
  Vector3d source_center = source_points.rowwise().sum() / num_points;
  Vector3d target_center = target_points.rowwise().sum() / num_points;

  // a) Calculate correlation matrix by a single matrix product (copy source and target points).
  //  Matrix3Xd source = source_points;
  //  Matrix3Xd target = target_points;
  //  source.colwise() -= source_center;
  //  target.colwise() -= target_center;
  //  Matrix3d corr_matrix = source * target.transpose();

  // b) Calculate correlation matrix as sum of matrices (no copy, no extra memory needed).
  Matrix3d corr_matrix = Matrix3d::Zero();
  for (int i = 0; i < num_points; i++) {
    corr_matrix +=
        (source_points.col(i) - source_center) * (target_points.col(i) - target_center).transpose();
  }

  // Singular-value decomposition of 3x3 correlation matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(corr_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const MatrixXd& U = svd.matrixU();
  const MatrixXd& V = svd.matrixV();

  // Compute orthonormal matrix that minimizes sum of squared distances between centered points.
  // Determinant of +1 is not yet guaranteed.
  // Determinant of -1 corresponds to a flip being optimal in least-squares sense.
  *rotation_matrix = V * U.transpose();

  // Guarantee that there is no flip -> increases residual
  if (!allow_flip && rotation_matrix->determinant() < 0) {
    Matrix3d Umod = U;
    Umod.col(2) = -U.col(2);
    *rotation_matrix = V * Umod.transpose();
  }

  // Compute the optimal translation vector given the optimal rotation matrix.
  // This translation vector is globally optimal (assuming there was no flip correction).
  *translation = target_center - (*rotation_matrix) * source_center;

  return true;
}

}  // namespace pnp
}  // namespace isaac
