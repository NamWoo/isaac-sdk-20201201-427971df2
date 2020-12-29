/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/pnp/gems/epnp/epnp.hpp"

#include <Eigen/SVD>

#include <cmath>
#include <utility>

namespace isaac {
namespace pnp {
namespace epnp {

// High-level function that implements the full EPnP pipeline
pnp::Status ComputeCameraPose(double focal_u, double focal_v, double principal_u,
                              double principal_v, const Matrix3Xd& points3,
                              const Matrix2Xd& points2, epnp::Result* result) {
  if (result == nullptr) {
    return pnp::Status::kErrorBadInputParams;
  }

  // Make sure focal lengths in pixels are non-zero.
  // There is no restriction on the principal point location.
  if (focal_u == 0 || focal_v == 0) {
    return pnp::Status::kErrorBadInputParams;
  }

  // At least 6 2D-3D point correspondences are required currently.
  // TODO: extend to 5 and 4 (requires complicated relinearization)
  if (points2.cols() != points3.cols() || points3.cols() < 6) {
    return pnp::Status::kErrorBadNumInputPoints;
  }

  // Compute control points in world coordinates. They represent a chosen 3D basis.
  constexpr double tol = 1e-3;
  result->ctl_points_world = ChooseBasis(points3, tol);

  // The dimensionality of the point cloud is the number of control points minus one because
  // the first control point is always the centroid.
  // Except when zero input points passed to ChooseBasis().
  result->input_dims = result->ctl_points_world.cols();
  if (result->input_dims) {
    result->input_dims--;
  }

  // Only planar and 3D input point cloud shapes are useful for camera pose estimation.
  if (result->input_dims < 2 || result->input_dims > 3) {
    return pnp::Status::kErrorDegenerateGeom;
  }

  // Compute barycentric coordinates of all input 3D points with respect to chosen basis.
  result->bary_coeffs = ComputeBaryCoords(points3, result->ctl_points_world);
  if (result->bary_coeffs.cols() == 0) {
    return pnp::Status::kErrorDegenerateGeom;
  }

  // Calculate space of the control point camera coordinates such that all
  // projection constraints are approximately satisified.
  if (!SolveProjConstraints(focal_u, focal_v, principal_u, principal_v, result->bary_coeffs,
                            points2, &result->proj_coeffs, &result->solution_basis,
                            &result->singular_values)) {
    return pnp::Status::kErrorDegenerateGeom;
  }

  // Precompute distances between control points in the world frame.
  VectorXd ctl_distances = ComputeDistances(result->ctl_points_world);

  // Construct calibration matrix needed for reprojections at residual calculation.
  Matrix3d calib_matrix;
  calib_matrix << focal_u, 0, principal_u, 0, focal_v, principal_v, 0, 0, 1;

  // Given the solution basis, solve for control point camera coordinates.
  // Solve for various possible dimensions of the solution space and retain
  // the resulting pose that has the least reprojection error.
  const int max_solution_dims = 3;  // Only 1,2,3 are supported
  double min_rss_error = Inf;
  result->rss_repr_errors = VectorXd::Constant(max_solution_dims, Inf);
  for (int k = 0; k < max_solution_dims; k++) {
    const int sol_dims = k + 1;

    // Compute weights that describe the concrete solution in the solution space.
    Vector4d weights = SolveControlPoints(result->solution_basis, sol_dims, ctl_distances);
    if (weights == Vector4d::Zero()) {
      continue;
    }

    // Calculate control points in camera frame given the weights and the base vectors.
    Matrix3Xd ctl_points_cam = ReshapeToMatrix3xN(result->solution_basis * weights);

    // Compute rigid-body transform between control points in world and in camera frame.
    // The resulting transformation is the sought camera pose.
    Matrix3d rotation;
    Vector3d translation;
    if (!pnp::RegisterPointsEucl(result->ctl_points_world, ctl_points_cam, &rotation,
                                 &translation)) {
      continue;
    }

    // Compute squared reprojection errors of the input points, given the computed camera pose.
    VectorXd repr_errors = pnp::ColwiseSquaredNorms(
        points2 - pnp::EuclideanFromHomogeneous(calib_matrix *
                                                ((rotation * points3).colwise() + translation)));

    // Calculate reprojection Residual Sum of Squares (RSS).
    double rss_error = repr_errors.sum();
    result->rss_repr_errors(k) = rss_error;

    // Keep the best solution in terms of reprojection RSS.
    if (rss_error < min_rss_error) {
      min_rss_error = rss_error;
      result->ctl_points_cam = ctl_points_cam;
      result->rotation = rotation;
      result->translation = translation;
      result->solution_dims = sol_dims;
    }
  }

  // Solution could not be found with any dimensionality of the solution space.
  if (min_rss_error == Inf) {
    return pnp::Status::kErrorDegenerateGeom;
  }

  return pnp::Status::kSuccess;
}

// Choose a suitable 3D basis to represent the input 3D points in (via barycentric coordinates).
// Principal Component Analysis of the 3D point cloud.
Matrix3Xd ChooseBasis(const Matrix3Xd& points3, double tol) {
  // The output basis will be represented by a small set of 3D control points (up to 4).
  Matrix3Xd ctl_points;
  ctl_points.resize(3, 0);

  // At least 1 input point is required for further calculations to make sense.
  const int num_points = points3.cols();
  if (num_points < 1) {
    return ctl_points;
  }

  // Calculate centroid of the point cloud - requires at least 1 point.
  Vector3d centroid = points3.rowwise().sum() / num_points;

  // Center the point cloud - requires at least 1 point
  Matrix3Xd centered_points3 = points3;
  for (int i = 0; i < num_points; i++) {
    centered_points3.col(i) -= centroid;
  }

  // Compute principal components.
  // Singular values are sorted in decreasing order.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(centered_points3, Eigen::ComputeFullU | Eigen::ComputeThinV);
  const VectorXd& singular_values = svd.singularValues();
  const MatrixXd& principal_directions = svd.matrixU();

  // Determine the number of non-vanishing principal components given a tolerance.
  int dims = 0;
  for (int i = 0; i < singular_values.rows(); i++) {
    if (singular_values(i) > tol) {
      dims++;
    }
  }

  // The output matrix is always of size 3-by-(dims+1) with the first column being the centroid
  // of the input point cloud.
  ctl_points = Matrix3Xd::Zero(3, dims + 1);
  ctl_points.col(0) = centroid;

  // Scale the other control points with respect to the centroid along the principal directions
  // such that their distance from the centroid equals the square root of the point variance.
  // This is only done for non-vanishing principal directions.
  const double factor = 1.0 / std::sqrt(static_cast<double>(num_points));
  for (int i = 0; i < dims; i++) {
    ctl_points.col(i + 1) = principal_directions.col(i) * factor * singular_values(i) + centroid;
  }

  return ctl_points;
}

// Compute barycentric coords of points with respect to a basis defined by 3 or 4 control points.
// Solves the linear problem A*X = B for X where:
// B is the 4xN matrix of homogeneous 3D points (3xN matrix points3 extended by a row of 1's)
// A is the 4x3 or 4x4 homogenenous matrix of control point coordinates,
// X is the 3xN or 4xN matrix of barycentric coordinates.
// Since the last columns of A and B are all 1's, the column sums of X are 1.
MatrixXd ComputeBaryCoords(const Matrix3Xd& points3, const Matrix3Xd& ctl_points) {
  MatrixXd bary_coords;
  bary_coords.resize(ctl_points.cols(), 0);
  if (points3.cols() && (ctl_points.cols() == 3 || ctl_points.cols() == 4)) {
    Matrix4Xd basis = pnp::HomogeneousFromEuclidean(ctl_points);

    // Solve the problem A*X = B
    bary_coords = basis.householderQr().solve(pnp::HomogeneousFromEuclidean(points3));
  }
  return bary_coords;
}

// Solve the projection equations via Singular-Value Decomposition and
// find the solution space of the control point camera coordinates.
bool SolveProjConstraints(double focal_u, double focal_v, double principal_u, double principal_v,
                          const MatrixXd& bary_coords, const Matrix2Xd points2,
                          MatrixXd* proj_coeffs, MatrixXd* sol_basis, VectorXd* singular_values) {
  if (proj_coeffs == nullptr || sol_basis == nullptr || singular_values == nullptr) {
    return false;
  }

  proj_coeffs->resize(0, 0);

  // At least 6 input points are required, each generating 2 equations.
  int num_points = points2.cols();
  if (num_points < 6) {
    return false;
  }
  if (bary_coords.cols() != num_points) {
    return false;
  }

  // Focal lengths should be non-zero.
  if (focal_u == 0 || focal_v == 0) {
    return false;
  }

  // Shorthands to make it easier to visually verify projection coefficients below.
  const double fu = focal_u;
  const double fv = focal_v;
  const double uc = principal_u;
  const double vc = principal_v;

  // Non-planar case with 4 control points (12 unknowns)
  if (bary_coords.rows() == 4) {
    // Construct the 2Nx12 coefficient matrix
    proj_coeffs->resize(2 * num_points, 12);
    for (int i = 0; i < num_points; i++) {
      // Shorthands to make it easier to check coefficient matrix
      const double u = points2(0, i);
      const double v = points2(1, i);
      const double b1 = bary_coords(0, i);
      const double b2 = bary_coords(1, i);
      const double b3 = bary_coords(2, i);
      const double b4 = bary_coords(3, i);
      proj_coeffs->row(2 * i + 0) << b1 * fu, 0, b1 * (uc - u), b2 * fu, 0, b2 * (uc - u), b3 * fu,
          0, b3 * (uc - u), b4 * fu, 0, b4 * (uc - u);
      proj_coeffs->row(2 * i + 1) << 0, b1 * fv, b1 * (vc - v), 0, b2 * fv, b2 * (vc - v), 0,
          b3 * fv, b3 * (vc - v), 0, b4 * fv, b4 * (vc - v);
    }

    // Planar case with 3 control points (9 unknowns)
  } else if (bary_coords.rows() == 3) {
    // Construct the 2Nx9 coefficient matrix
    proj_coeffs->resize(2 * num_points, 9);
    for (int i = 0; i < num_points; i++) {
      // Shorthands to make it easier to check coefficient matrix
      const double u = points2(0, i);
      const double v = points2(1, i);
      const double b1 = bary_coords(0, i);
      const double b2 = bary_coords(1, i);
      const double b3 = bary_coords(2, i);
      proj_coeffs->row(2 * i + 0) << b1 * fu, 0, b1 * (uc - u), b2 * fu, 0, b2 * (uc - u), b3 * fu,
          0, b3 * (uc - u);
      proj_coeffs->row(2 * i + 1) << 0, b1 * fv, b1 * (vc - v), 0, b2 * fv, b2 * (vc - v), 0,
          b3 * fv, b3 * (vc - v);
    }

    // Cases that are not solved with the current implementation.
  } else {
    return false;
  }

  // Solve the homogeneous linear problem by SVD
  // Matrix V is 9x9 (planar case) or 12x12 (non-planar case)
  // There are 9 (planar case) or 12 singular values  (non-planar case)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(*proj_coeffs, Eigen::ComputeThinU | Eigen::ComputeFullV);
  *singular_values = svd.singularValues();
  const MatrixXd& singular_vectors = svd.matrixV();

  // Output the last 4 columns of V in reverse order
  sol_basis->resize(singular_vectors.rows(), 4);
  for (int i = 0; i < 4; i++) {
    sol_basis->col(i) = singular_vectors.col(singular_vectors.cols() - i - 1);
  }

  // Put singular values in increasing order
  for (int i = 0; i < singular_values->size() / 2; i++) {
    std::swap((*singular_values)(i), (*singular_values)(singular_values->size() - 1 - i));
  }

  return true;
}

// Convert a solution vector of control point coordinates of length 3*C to a 3xC matrix for C=3,4
// such that the input vector contains the elements of the output matrix column-wise.
Matrix3Xd ReshapeToMatrix3xN(const VectorXd& vec) {
  Matrix3Xd matrix;
  if (vec.size() % 3) {
    return matrix;  // return a 3x0 matrix
  }
  matrix.resize(3, vec.size() / 3);
  for (int i = 0; i < matrix.cols(); i++) {
    matrix.col(i) = vec.block(3 * i, 0, 3, 1);
  }

  return matrix;
}

// Project points directly given in camera coordinates to the image
Matrix2Xd ProjectPoints(const Matrix3d& calib_matrix, const Matrix3Xd& points3) {
  Matrix2Xd points2;
  points2.resize(2, points3.cols());
  for (int i = 0; i < points3.cols(); i++) {
    Vector3d proj = calib_matrix * points3.col(i);
    if (proj(2) == 0) {
      // Avoids NaNs due to 0/0.
      points2(0, i) = Inf;
      points2(1, i) = Inf;
    } else {
      // Division by homogeneous coordinate.
      points2(0, i) = proj(0) / proj(2);
      points2(1, i) = proj(1) / proj(2);
    }
  }
  return points2;
}

// Compute distances between all possible pairs (i,j) of input points (in matrix columns).
VectorXd ComputeDistances(const MatrixXd& points) {
  int num_points = points.cols();
  if (num_points < 2) {
    return VectorXd(0);
  }

  VectorXd distances(num_points * (num_points - 1) / 2);
  int k = 0;
  for (int i = 0; i < points.cols(); i++) {
    for (int j = i + 1; j < points.cols(); j++) {
      distances[k++] = (points.col(i) - points.col(j)).norm();
    }
  }

  return distances;
}

// Compute the weights that correspond to a concrete solution for the control point camera
// coordinates in the basis given as input.
Vector4d SolveControlPoints(const MatrixXd& sol_basis, int sol_dims, const VectorXd& distances) {
  Vector4d weights = Vector4d::Zero();
  if (sol_basis.cols() != 4) {
    return weights;
  }
  if (sol_basis.rows() != 12 && sol_basis.rows() != 9) {
    return weights;
  }

  // non-planar case
  if (sol_basis.rows() == 12 && distances.size() != 6) {
    return weights;
  }

  // planar case
  if (sol_basis.rows() == 9 && distances.size() != 3) {
    return weights;
  }

  // Planar and non-planar case for 1-dimensional solution space
  if (sol_dims == 1) {
    // Only use singular vector corresponding to the least singular value.
    // Solution space is 1-dimensional -> only need to find a single global scale.

    // Calculate the unscaled camera coordinates of the control points.
    Matrix3Xd v = ReshapeToMatrix3xN(sol_basis.col(0));

    // Calculate distances between the unscaled control points.
    VectorXd distances_cam = ComputeDistances(v);

    // Calculate the unknown scale from the distances between control points in the world frame
    // and the corresponding distances in the camera frame.
    double scale = distances.dot(distances_cam) / distances_cam.dot(distances_cam);

    // Find sign of the scale such that a majority of control points are in front of the camera.
    int num_points_behind = (v.row(2).array() < 0).count();
    if (num_points_behind > 1) {
      scale = -scale;
    }

    weights(0) = scale;

    // Planar and non-planar for 2-dimensional solution space
  } else if (sol_dims == 2) {
    // Reshape the two basis vectors of the solution space into two 3xN matrices.
    // The sought matrix ctl_points_cam is a linear combination of these matrices.
    // ctl_points_cam = w1*v1 + w2*v2, where the weights w1 and w2 are still unknown.
    Matrix3Xd v1 = ReshapeToMatrix3xN(sol_basis.col(0));
    Matrix3Xd v2 = ReshapeToMatrix3xN(sol_basis.col(1));

    // Looking for the linear combination preserving the distances between control points.
    // This is a non-linear problem in the linear combination weights (w1,w2).
    // Strategy:
    //   (Step1) Solve for (w11,w22,w12) = (w1*w1, w2*w2, w1*w2) (linearized problem)
    //   (Step2) Extract (w1,w2) from (w11,w22,w12).
    // Both steps are over-determined problems.

    // Construct coefficient matrix of linearized problem (see Eq.13 in the paper).
    // This matrix is 6x4 in the non-planar case.
    const int num_points = v1.cols();
    const int num_equations = num_points * (num_points - 1) / 2;
    int k = 0;
    MatrixXd coeff_matrix(num_equations, 3);
    for (int i = 0; i < num_points; i++) {
      for (int j = i + 1; j < num_points; j++) {
        Vector3d d1 = v1.col(i) - v1.col(j);
        Vector3d d2 = v2.col(i) - v2.col(j);
        coeff_matrix.row(k++) << d1.dot(d1), d2.dot(d2), 2 * d1.dot(d2);
      }
    }

    // Step1: Compute least squares solution to the linearized problem.
    VectorXd squared_distances = distances.cwiseProduct(distances);
    VectorXd lin_sol =
        coeff_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(squared_distances);

    // Solution of the linearized problem.
    const double w11 = lin_sol(0);
    const double w22 = lin_sol(1);
    const double w12 = lin_sol(2);

    // Step2: Calculate weights w1 and w2 (denoted beta_i in the EPnP paper)
    // from the non-linear constraints (w11,w22,w12) = (w1*w1, w2*w2, w1*w2).
    // There are always two solutions, (w1,w2) and (-w1,-w2) -> sign is solved further below.
    // Over-constrained quadratic problem - least-squares solution has no simple closed form.
    // Empirical tests show that solution order largely affects pose accuracy.
    // Strategy: we exactly match the larger magnitude wij and hope for less overall alg. error.
    double w1, w2;
    w1 = w2 = 0;
    if (w11 >= w22 && w11 > 0) {
      w1 = std::sqrt(w11);
      if (w22 > std::abs(w12)) {
        w2 = std::sqrt(w22);
      } else {
        w2 = w12 / w1;
      }
    } else if (w11 <= w22 && w22 > 0) {
      w2 = std::sqrt(w22);
      if (w11 > std::abs(w12)) {
        w1 = std::sqrt(w11);
      } else {
        w1 = w12 / w2;
      }
    }

    // There are always two solutions: (w1,w2) and (-w1,-w2) but
    // one corresponds to control points behind the camera.
    // Find sign such that a majority all control points are in front of the camera.
    VectorXd ctl_point_depths = w1 * v1.row(2) + w2 * v2.row(2);
    int num_points_behind = (ctl_point_depths.array() < 0).count();
    if (num_points_behind > int{num_points / 2}) {
      weights << -w1, -w2, 0, 0;
    } else {
      weights << w1, w2, 0, 0;
    }

    // Non-planar case for 2-dimensional solution space
  } else if (sol_dims == 3 && sol_basis.rows() == 12) {
    // Reshape the 3 basis vectors of the solution space into three 3xN matrices.
    // The sought matrix ctl_points_cam is a linear combination of these matrices.
    // ctl_points_cam = w1*v1 + w2*v2 + w3*v3, where the weights (w1,w2,w3) are still unknown.
    Matrix3Xd v1 = ReshapeToMatrix3xN(sol_basis.col(0));
    Matrix3Xd v2 = ReshapeToMatrix3xN(sol_basis.col(1));
    Matrix3Xd v3 = ReshapeToMatrix3xN(sol_basis.col(2));

    // Looking for the linear combination preserving the distances between control points.
    // This is a non-linear problem in the linear combination weights (w1,w2,w3).
    // Strategy:
    //   (Step1) Solve for (w11,w22,w33,w12,w13,w23) = (w1*w1, w2*w2, w3*w3, w1*w2, w1*w3, w2*w3)
    //           (linearized problem).
    //   (Step2) Extract (w1,w2,w3) from (w11,w22,w33,w12,w13,w23).

    // Step1: Construct coefficient matrix of linearized problem (Case N=3 in the paper).
    // This matrix is 6x6 in the non-planar case.
    const int num_points = v1.cols();
    const int num_equations = num_points * (num_points - 1) / 2;
    int k = 0;
    MatrixXd coeff_matrix(num_equations, 6);
    for (int i = 0; i < num_points; i++) {
      for (int j = i + 1; j < num_points; j++) {
        Vector3d d1 = v1.col(i) - v1.col(j);
        Vector3d d2 = v2.col(i) - v2.col(j);
        Vector3d d3 = v3.col(i) - v3.col(j);
        coeff_matrix.row(k++) << d1.dot(d1), d2.dot(d2), d3.dot(d3), 2 * d1.dot(d2), 2 * d1.dot(d3),
            2 * d2.dot(d3);
      }
    }

    // Step1: Compute least squares solution to the linearized problem.
    VectorXd squared_distances = distances.cwiseProduct(distances);
    VectorXd lin_sol =
        coeff_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(squared_distances);

    // Solution of the linearized problem.
    const double w11 = lin_sol(0);
    const double w22 = lin_sol(1);
    const double w33 = lin_sol(2);
    const double w12 = lin_sol(3);
    const double w13 = lin_sol(4);
    const double w23 = lin_sol(5);

    // Calculate weights w1, w2, w3 from the non-linear constraints
    // (w11, w22, w33, w12, w13, w23) = (w1*w1, w2*w2, w3*w3, w1*w2, w1*w3, w2*w3).
    // There are always two solutions (w1,w2,w3) and (-w1,-w2,-w3) -> sign solved further below.
    // Over-constrained quadratic problem - leats-squares solution has no simple closed form.
    // Empirical tests show that solution order largely affects pose accuracy.
    // Strategy: we exactly match the larger magnitude wij and hope for less overall alg. error.
    double w1 = 0, w2 = 0, w3 = 0;
    if (w11 > w22 && w11 > w33 && w11 > 0) {
      w1 = std::sqrt(w11);
      if (w22 > std::abs(w12)) {
        w2 = std::sqrt(w22);
      } else {
        w2 = w12 / w1;
      }
      if (w33 > std::abs(w13)) {
        w3 = std::sqrt(w33);
      } else {
        w3 = w13 / w1;
      }
    } else if (w22 > w11 && w22 > w33 && w22 > 0) {
      w2 = std::sqrt(w22);
      if (w11 > std::abs(w12)) {
        w1 = std::sqrt(w11);
      } else {
        w1 = w12 / w2;
      }
      if (w33 > std::abs(w23)) {
        w3 = std::sqrt(w33);
      } else {
        w3 = w23 / w2;
      }
    } else if (w33 > w11 && w33 > w22 && w33 > 0) {
      w3 = std::sqrt(w33);
      if (w11 > std::abs(w13)) {
        w1 = std::sqrt(w11);
      } else {
        w1 = w13 / w3;
      }
      if (w22 > std::abs(w23)) {
        w2 = std::sqrt(w22);
      } else {
        w2 = w23 / w3;
      }
    }

    // There are always two solutions: (w1,w2,w3) and (-w1,-w2,-w3) but
    // one corresponds to control points behind the camera.
    // Find sign such that a majority all control points are in front of the camera.
    VectorXd ctl_point_depths = w1 * v1.row(2) + w2 * v2.row(2) + w3 * v3.row(2);
    int num_points_behind = (ctl_point_depths.array() < 0).count();
    if (num_points_behind > int{num_points / 2}) {
      weights << -w1, -w2, -w3, 0;
    } else {
      weights << w1, w2, w3, 0;
    }
  }

  // TODO: planar N=3 case using relinearization
  // TODO: non-planar N=4 (affine camera) case using relinearization
  // planar N=4 case is underconstrained = ambiguous (3 equations but 4 unknowns)

  return weights;
}

}  // namespace epnp
}  // namespace pnp
}  // namespace isaac
