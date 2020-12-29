/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

// Public header for camera pose estimation (PnP) algorithms given 2D-3D correspondences.

#include <vector>

#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace pnp {

// Return status for different PnP functions.
enum class Status {
  kSuccess,                 // Pose estimation succeeded.
  kErrorBadInputParams,     // Input parameters of the camera / algorithm are invalid.
  kErrorBadNumInputPoints,  // Unacceptable #points or numbers of 2D and 3D points do not match.
  kErrorDegenerateGeom,     // Pose estimation is unstable given the point/camera setup.
  kErrorNullPointer         // Unallocated output parameter.
};

// EPnP algorithm: given the camera intrinsics, compute the pose of a pinhole camera
// from at least 6 2D-3D point correspondences without outliers.
// The algorithm handles both 3D and planar point arrangements.
// Assumes no lens distortions, you need to un-distort points in advance.
//  focal_u,v      Relative focal lengths in horizontal / vertical pixel sizes from calibration
//  principal_u,v  Principal point coordinates in pixels from calibration
//  points3        Input 3D points as 3xN matrix (N>=6)
//  points2        Input 2D points as 2xN matrix (in corresponding order to 3D points)
//  pose           Output camera pose: rigid transformation that maps 3D points from
//                 the world frame into the camera frame.
Status ComputeCameraPoseEpnp(const Matrix3Xd& points3, const Matrix2Xd& points2, double focal_u,
                             double focal_v, double principal_u, double principal_v, Pose3d* pose);

// RANSAC formula: Calculate the number of RANSAC rounds necessary to sample at least a single
// uncontaminated sample set with a certain success rate given the expected ratio of outliers.
//   success_rate   Required probability in the range (0,0.9999] of finding the solution.
//                  As success_rate tends to 1.0, the number of experiments tends to Inf.
//   outlier_ratio  Maximum expected ratio of outliers in the open interval [0,0.9]
//   sample_size    The minimum number of samples necessary to fit the model (at least 1).
// Returns the number of experiments (RANSAC iterations) from the formula.
// The input parameters are capped at their respective limits internally.
unsigned int EvaluateRansacFormula(float success_rate, float outlier_ratio,
                                   unsigned int sample_size);

// Pose hypothesis returned by RANSAC-based pose estimation.
struct PoseHypothesis {
  Pose3d pose;                    // Camera pose preferably after refitting to all inliers
  double score;                   // Score of the pose (prior to refitting)
  std::vector<unsigned> inliers;  // Indices of all inliers for this hypothesis
};

// RANSAC with 6-point EPnP for robust camera pose estimation.
// Given the camera intrinsics, compute the pose of a pinhole camera from at least 6 2D-3D
// point correspondences possibly contaminated with gross outliers.
//  points3           Input 3D points as 3xN matrix (N>=6)
//  points2           Input 2D points as 2xN matrix (in corresponding order to 3D points)
//  focal_u,v         Relative focal lengths in horizontal / vertical pixel sizes from calibration
//  principal_u,v     Principal point coordinates in pixels from calibration
//  num_rounds        Number of RANSAC iterations or experiments to run.
//  ransac_threshold  RANSAC threshold in terms of reprojection error in pixels.
//  max_top_poses     Maximum number of pose hypotheses to return.
//  seed              Integer seed for random number generation.
// Returns at most top-K pose hypotheses in decreasing order of score. Each pose hypothesis has
// min 6 inliers and is a rigid transformation mapping 3D points from world to camera frame.
//
// Algorithm: RANSAC soft-scoring scheme is used: only inlier 2D-3D matches contribute to the score,
// by +exp(-e^2/(2*sigma^2)) where e is the reprojection error norm (in pixels) and
// sigma = ransac_threshold / 3 (pixels).
// A match is an inlier if its reprojection error e <= ransac_threshold.
// If reprojection error e = 0                     -> contribution is 1.0
// If reprojection error e = 0.33*ransac_threshold -> contribution is 0.607
// If reprojection error e = 0.66*ransac_threshold -> contribution is 0.135
// If reprojection error e > ransac_threshold      -> contribution is 0
std::vector<PoseHypothesis> ComputeCameraPoseEpnpRansac(const Matrix3Xd& points3,
                                                        const Matrix2Xd& points2, double focal_u,
                                                        double focal_v, double principal_u,
                                                        double principal_v, unsigned num_rounds,
                                                        double ransac_threshold,
                                                        unsigned max_top_poses, unsigned seed = 0);

}  // namespace pnp
}  // namespace isaac
