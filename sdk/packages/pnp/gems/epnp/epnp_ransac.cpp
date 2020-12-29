/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/pnp/gems/epnp/epnp_ransac.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "packages/pnp/gems/generic/utils.hpp"

namespace isaac {
namespace pnp {
namespace epnp {

// Print array of RANSAC seed indices (indices of a subset of original input points).
// A RANSAC seed is the subset of input points used to generate a model hypothesis.
std::ostream& operator<<(std::ostream& os, const std::array<unsigned, 6>& seed) {
  for (unsigned i : seed) {
    os << " " << seed[i];
  }
  return os;
}

// Print a RANSAC pose hypothesis.
// ModelType     The type of the model to fit to the data.
// sample_size   Number of input data items required to generate a model hypothesis in RANSAC.
template <typename ModelType, unsigned sample_size>
std::ostream& operator<<(std::ostream& os,
                         const isaac::pnp::RansacHypothesis<ModelType, sample_size>& hypothesis) {
  os << "score=" << hypothesis.score << " inliers=" << hypothesis.inliers.size();
  return os;
}

// Entry point for input data and RANSAC threshold.
// Does not check the data and parameters. Checks should be done inside checkInput().
EpnpRansacAdaptor::EpnpRansacAdaptor(const isaac::Matrix3Xd& points3,
                                     const isaac::Matrix2Xd& points2, double focal_u,
                                     double focal_v, double principal_u, double principal_v,
                                     double ransac_threshold)
    : points3_(points3), points2_(points2) {
  // Camera calibration matrix (needed for reprojection at scoring)
  calib_matrix_ << focal_u, 0, principal_u, 0, focal_v, principal_v, 0, 0, 1;

  // Variables used for a RANSAC soft-scoring scheme
  // Each point contributes to the score by exp(-ei^2/(2*sigma^2))
  // where ei is the norm of the reprojection error for point i.
  // 0 reprojection error -> contributes by 1.0
  // An error of 0.33*threshold -> contributes by ~0.607
  // An error of 0.66*threshold -> contributes by ~0.135
  // An error of  >=  threshold -> contributes by exactly 0 (clamped from 0.01)
  threshold_squared_ = ransac_threshold * ransac_threshold;
  double sigma_squared = threshold_squared_ / 9;  // sigma = threshold / 3
  score_coeff_ = -0.5 / sigma_squared;
}

// Check the input data and parameters passed earlier to the constructor.
// No need to check ransac_threshold, only its square is stored internally and 0 is allowed.
bool EpnpRansacAdaptor::checkInput() const {
  // Focal lengths have to be non-zero.
  if (calib_matrix_(0, 0) == 0 || calib_matrix_(1, 1) == 0) {
    return false;
  }

  // Number of 2D and 3D points has to match.
  if (points2_.cols() != points3_.cols() || points2_.cols() < kSampleSize) {
    return false;
  }

  return true;
}

// Compute pose from all inliers.
// inliers is a list of indices of inliers within the original input 2D-3D point matches.
bool EpnpRansacAdaptor::refit(const std::vector<unsigned>& inliers, ModelType* pose) const {
  // Create a copy of inliers only.
  isaac::Matrix3Xd points3 = SliceMatrixColumns(points3_, inliers);
  isaac::Matrix2Xd points2 = SliceMatrixColumns(points2_, inliers);
  // Compute camera pose using EPnP based on all inliers.
  pnp::Status status =
      ComputeCameraPoseEpnp(points3, points2, calib_matrix_(0, 0), calib_matrix_(1, 1),
                            calib_matrix_(0, 2), calib_matrix_(1, 2), pose);
  return (status == Status::kSuccess);
}

// Generate a single pose hypothesis from a seed by using the EPnP algorithm, then
// score the hypothesis using a soft-scoring scheme.
EpnpRansacAdaptor::ResultType EpnpRansacAdaptor::evaluate(
    const std::array<unsigned, kSampleSize>& seed) const {
  ResultType hypothesis;
  hypothesis.seed = seed;
  hypothesis.score = -1.0;  // negative score = hypothesis has not been evaluated

  // Select seed points by slicing the matrix of all input points.
  Pose3d pose;
  isaac::Matrix3Xd points3 = SliceMatrixColumns(points3_, hypothesis.seed);
  isaac::Matrix2Xd points2 = SliceMatrixColumns(points2_, hypothesis.seed);

  // Compute pose hypothesis from seed points using EPnP.
  pnp::Status status =
      ComputeCameraPoseEpnp(points3, points2, calib_matrix_(0, 0), calib_matrix_(1, 1),
                            calib_matrix_(0, 2), calib_matrix_(1, 2), &hypothesis.model);
  if (status != Status::kSuccess) {
    return hypothesis;
  }

  // Pose hypothesis scoring: project all input points to the image and
  // calculate score contribution in function of the reprojection error.
  hypothesis.score = 0;
  isaac::Matrix3d proj_matrix = calib_matrix_ * hypothesis.model.rotation.matrix();
  isaac::Vector3d translation = calib_matrix_ * hypothesis.model.translation;
  for (int i = 0; i < points3_.cols(); i++) {
    // Project point to the image.
    isaac::Vector3d hom_proj = proj_matrix * points3_.col(i) + translation;

    // Points behind the camera and points on the principal plane do not contribute to the score.
    if (hom_proj(2) <= 1e-6) {
      continue;
    }

    // Divide by homogeneous coordinate. Division by zero avoided above.
    isaac::Vector2d proj{hom_proj(0) / hom_proj(2), hom_proj(1) / hom_proj(2)};

    // Calculate squared reprojection error.
    double squared_repr_error = (proj - points2_.col(i)).squaredNorm();

    // Identify inliers and accumulate score (outliers do not contribute).
    if (squared_repr_error <= threshold_squared_) {
      // Scoring function as explained at the function declaration.
      hypothesis.score += exp(score_coeff_ * squared_repr_error);
      hypothesis.inliers.push_back(i);
    }
  }

  return hypothesis;
}

// Decide of two poses (a and b) are similar within tolerance parameters.
bool EpnpRansacAdaptor::isSimilar(const ModelType& model1, const ModelType& model2,
                                  double max_distance, double max_angle_degrees) {
  // Calculate difference in orientation.
  SO3d delta_rot = model1.rotation.inverse() * model2.rotation;
  double angle = std::fmod(RadToDeg(std::fabs(delta_rot.angle())), 360.0);
  angle = std::min(angle, 360.0 - angle);
  if (angle > max_angle_degrees) {
    return false;
  }

  // Calculate difference in position.
  // Convert world origin in camera frame to camera position in world frame. This can make
  // a significant difference in case of a moving camera far away from world origin.
  isaac::Vector3d pos1 = -model1.rotation.matrix().transpose() * model1.translation;
  isaac::Vector3d pos2 = -model2.rotation.matrix().transpose() * model2.translation;
  double distance = (pos1 - pos2).squaredNorm();
  if (distance > max_distance * max_distance) {
    return false;
  }

  return true;
}

}  // namespace epnp
}  // namespace pnp
}  // namespace isaac
