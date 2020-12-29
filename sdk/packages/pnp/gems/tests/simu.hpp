/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

#include <random>
#include <vector>

#include "packages/pnp/gems/generic/utils.hpp"

namespace isaac {
namespace pnp {

// Single global random engine used in all simulations.
std::mt19937& GlobalRandomEngine();

// Generate random matrix by drawing each element from the standard normal distribution.
template <typename MatrixType>
MatrixType RandomGaussianMatrix(unsigned rows, unsigned cols) {
  MatrixType matrix(rows, cols);
  std::normal_distribution<typename MatrixType::Scalar> distribution(0, 1);
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      matrix(i, j) = distribution(GlobalRandomEngine());
    }
  }
  return matrix;
}

// Generate random matrix with each element uniformly distributed in [min_value, max_value].
template <typename MatrixType>
MatrixType RandomUniformMatrix(unsigned rows, unsigned cols,
                               typename MatrixType::Scalar min_value = -1.0,
                               typename MatrixType::Scalar max_value = 1.0) {
  MatrixType matrix(rows, cols);
  std::uniform_real_distribution<typename MatrixType::Scalar> distribution(min_value, max_value);
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      matrix(i, j) = distribution(GlobalRandomEngine());
    }
  }
  return matrix;
}

// Generate random vector with each element uniformly distributed in [min_value, max_value].
template <typename VectorType>
VectorType RandomVector(unsigned size, typename VectorType::Scalar min_value = -1.0,
                        typename VectorType::Scalar max_value = 1.0) {
  VectorType vec(size, 1);
  std::uniform_real_distribution<typename VectorType::Scalar> distribution(min_value, max_value);
  for (int i = 0; i < vec.size(); i++) {
    vec(i) = distribution(GlobalRandomEngine());
  }
  return vec;
}

// Minimal class to draw random 3D points from a general 3D Gaussian distribution.
// The Gaussian is modeled by its mean, 3D orientation and deviations along its principal axes.
// Mathematically, the covariance matrix can be composed from the orientation and deviations, and
// they can be obtained from the covariance matrix by eigendecomposition (or SVD).
class Gaussian3 {
 private:
  Matrix3d orientation_;  // Orthonormal columns are the principal directions
  Vector3d deviations_;   // Gaussian deviation along each principal direction
  Vector3d mean_;         // Mean of the distribution (center of the Gaussian blob)

 public:
  Gaussian3() : orientation_(Matrix3d::Identity()), deviations_(1, 1, 1), mean_(0, 0, 0) {}

  // Change Gaussian parameters.
  void setOrientation(const Vector3d& angle_axis) {
    orientation_ = MatrixFromAngleAxis(angle_axis);
  }
  void setDeviations(const Vector3d& deviations) { deviations_ = deviations; }
  void setMean(const Vector3d& mean) { mean_ = mean; }

  // Access Gaussian parameters.
  const Vector3d& deviations() const { return deviations_; }
  const Vector3d& mean() const { return mean_; }
  const Matrix3d& orientation() const { return orientation_; }

  // Generate 3D samples from a distribution and return them in columns of a matrix.
  Matrix3Xd generate(unsigned num_samples) const {
    Matrix3d scaling_matrix;
    scaling_matrix << deviations_(0), 0, 0, 0, deviations_(1), 0, 0, 0, deviations_(2);

    // Transform a sample drawn from standard normal distribution to a sample drawn from
    // a zero-mean Gaussian distribution with the prescribed covariance matrix.
    Matrix3Xd samples =
        orientation_ * scaling_matrix * RandomGaussianMatrix<Matrix3Xd>(3, num_samples);

    // Translate the Gaussian blob to its prescribed mean.
    for (int i = 0; i < samples.cols(); i++) {
      samples.col(i) += mean_;
    }

    return samples;
  }
};

// Simulate a perspective camera with fixed intrinsics but random orientation and position.
// Orientation is generated via an angle-axis with uniform random direction in spherical
// coordinates and a uniform random rotation angle.
// Position coordinates are uniformly distributed in [-pos_range, pos_range].
//  width, height    Image width and height in pixels.
//  focal            Focal length in pixels (square pixels assumed).
//  pos_range        Maximum distance of each positional coordinate from 0.
// Returns the generated camera.
Camera GenerateRandomCamera(int width, int height, double focal, double pos_range = 100.0);

// Generate random 2D points uniformly distributed in the image.
// Continuous (floating point) coordinate ranges: x:[0,width] and y:[0,height].
//  num_points       Number of random points to generate.
//  width,height     Resolution of the image in pixels.
// Returns the generated 2D points as columns of a 2-by-num_points matrix.
Matrix2Xd GenerateImagePoints(unsigned num_points, unsigned width, unsigned height);

// Generate random 2D-3D point matches given a perspective view such that all 3D points are within
// the camera FoV between the near and the far planes.
//  num_points       Number of random points to generate.
//  camera           Parameters of the perspective camera.
//  near,far         Depth of the near and plane (both positive, near < far).
//  points3          3-by-num_points matrix of output 3D points in world coordinates.
//  points2          2-by-num_points matrix of respective output projections in pixel coordinates.
void GenerateFovPoints(unsigned num_points, const Camera& camera, double near, double far,
                       Matrix3Xd* points3, Matrix2Xd* points2);

// Generate random 2D-3D point matches given a perspective view such that all 3D points lie
// within the camera FoV on a plane in front of the camera. The depth of the plane and
// its angle with respect to the image plane is fixed but its exact orientation is random.
// Optionally add Gaussian depth noise to each point to simulate deviations from the plane.
// Note that this keeps the generated 2D-3D matches in perfect correspondence.
// The depth noise is measured as a distance along ray and is clamped into the range
// [-3*deviation, 3*deviation] around the plane to allow for deterministic checks.
// The max distance of points from the plane is 3*deviation and this decreases for slanted planes.
// The depth is also capped at 1e-3 so no point is ever moved behind the camera by noise.
//  num_points  Number of random point matches to generate.
//  camera      Parameters of the perspective camera.
//  depth       Depth of the plane along the optical axis.
//  angle       Angle in degrees between the image plane and the world plane.
//              Must be in the range [0,85], 0 degrees corresponding to a fronto-parallel plane.
//  deviation   Deviation of the additive Gaussian depth noise along the ray.
//              Noise is switched off when deviation = 0.
//  points3     Output 3-by-num_points matrix of generated 3D points in world coordinates.
//  points2     Output 2-by-num_points matrix of respective projections in pixel coordinates.
//  plane       Output homogeneous plane coefficients of the generated plane.
void GenerateFovPointsPlanar(unsigned num_points, const Camera& camera, double depth, double angle,
                             double deviation, Matrix3Xd* points3, Matrix2Xd* points2,
                             Vector4d* plane);

// Generate outlier 2D-3D point matches and insert them into the lists of input 2D/3D points.
// 3D points are sampled uniformly in an axis-aligned box of size 'radius' around the camera.
// 2D points are sampled uniformly within the image, independently of the sampled 3D points.
// There is no guarantee that each generated pair is an outlier but the probability is near 1.
//  num_outliers  The number of outliers to generate and mix with the input.
//  camera        Parameters of the perspective camera.
//  radius        Maximum absolute value for the generated 3D point coordinates.
//  points3       List of input / output 3D points.
//  points2       List of respective input / output 2D projections (pixel coordinates).
//  shuffle       If shuffle = false, outliers are appended at the end of points3 and points2.
//                If shuffle = true, the outliers are inserted into random columns.
//                The order of the input points is kept in both cases but are
//                randomly interleaved with outliers when shuffle = true.
// Returns the output column index list of the added outliers.
std::vector<int> InsertOutliers(unsigned num_outliers, const Camera& cam, double radius,
                                Matrix3Xd* points3, Matrix2Xd* points2, bool shuffle = true);

}  // namespace pnp
}  // namespace isaac
