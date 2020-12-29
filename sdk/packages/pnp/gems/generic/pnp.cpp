/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/pnp/gems/pnp.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <vector>

#include "packages/pnp/gems/epnp/epnp_ransac.hpp"

namespace isaac {
namespace pnp {

// EPnP algorithm: compute the 3D (6-DoF) pose of a calibrated pinhole camera from >=6 2D-3D point
// correspondences in either 3D or in planar arrangement.
// Assumes corrected lens distortions.
Status ComputeCameraPoseEpnp(const Matrix3Xd& points3, const Matrix2Xd& points2, double focal_u,
                             double focal_v, double principal_u, double principal_v, Pose3d* pose) {
  if (pose == nullptr) {
    return Status::kErrorNullPointer;
  }

  epnp::Result result;
  Status status = epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v, points3,
                                          points2, &result);
  if (status == Status::kSuccess) {
    Vector3d angle_axis = AngleAxisFromMatrix(result.rotation);
    pose->rotation = SO3d::FromAngleAxis(angle_axis.norm(), angle_axis);
    pose->translation = result.translation;
  }

  return status;
}

// RANSAC formula: Calculate the number of RANSAC rounds necessary to sample at least a single
// uncontaminated sample set with a certain success rate given the expected ratio of outliers.
unsigned int EvaluateRansacFormula(float success_rate, float outlier_ratio,
                                   unsigned int sample_size) {
  constexpr float max_success_rate = 0.9999;
  constexpr float max_outlier_ratio = 0.9;

  // theor.limits: sample_size > 0, 0 < outlierRate < 1, 0 <= success_rate < 1
  if (success_rate < 0) {
    success_rate = 0;
  } else if (success_rate > max_success_rate) {
    success_rate = max_success_rate;
  }

  if (outlier_ratio <= 0) {
    return 1;
  } else if (outlier_ratio > max_outlier_ratio) {
    outlier_ratio = max_outlier_ratio;
  }

  if (sample_size < 1) {
    sample_size = 1;
  }

  // RANSAC formula
  double good_sample_prob = pow(1 - outlier_ratio, sample_size);
  return ceil(log(1 - success_rate) / log(1.0 - good_sample_prob));
}

// RANSAC with 6-point EPnP for robust camera pose estimation.
// Returns top K different poses in an iterable priority queue.
std::vector<PoseHypothesis> ComputeCameraPoseEpnpRansac(const Matrix3Xd& points3,
                                                        const Matrix2Xd& points2, double focal_u,
                                                        double focal_v, double principal_u,
                                                        double principal_v, unsigned num_rounds,
                                                        double ransac_threshold,
                                                        unsigned max_top_poses, unsigned seed) {
  // RANSAC sampler for 6-point pose.
  pnp::DefaultRansacSampler<6> sampler(points3.cols(), seed);

  // RANSAC adaptor for the EPnP algorithm: pose-, EPnP- and scoring-specific part of RANSAC.
  epnp::EpnpRansacAdaptor adaptor(points3, points2, focal_u, focal_v, principal_u, principal_v,
                                  ransac_threshold);

  // Invoke generic RANSAC-TopK algorithm using this core algorithm.
  auto top_poses = pnp::Ransac(adaptor, num_rounds, max_top_poses, &sampler);

  // Copy the list of top pose hypotheses returned by the algorithm.
  // This copy allows public and private interfaces to differ.
  std::vector<PoseHypothesis> result;
  for (const pnp::RansacHypothesis<Pose3d, 6>& src : top_poses) {
    PoseHypothesis dst;
    dst.pose = src.model;
    dst.score = src.score;
    dst.inliers = src.inliers;
    result.push_back(dst);
  }
  return result;
}

}  // namespace pnp
}  // namespace isaac
