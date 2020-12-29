/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

#include <array>
#include <vector>

#include "packages/pnp/gems/epnp/epnp.hpp"
#include "packages/pnp/gems/generic/ransac.hpp"

namespace isaac {
namespace pnp {
namespace epnp {

// Adaptor for the EPnP pose estimation algorithm to be used in a RANSAC scheme.
// The purpose of the adaptor has to define the model type to fit to the data,
// the number of samples necessary to fit the model, to implement both model fitting
// and the concrete RANSAC scoring method.
class EpnpRansacAdaptor {
 public:
  // Number of samples to be used to fit the model in each RANSAC round.
  constexpr static unsigned kSampleSize = 6;

  // Type of the model to fit.
  using ModelType = Pose3d;

  // Hypothesis type resulting from each RANSAC round.
  using ResultType = isaac::pnp::RansacHypothesis<ModelType, kSampleSize>;

  // Intrinsic camera parameters, 2D-3D point matches and RANSAC inlier threshold.
  //  focal_u,v         Relative focal lengths in horizontal / vertical pixel sizes from calibration
  //  principal_u,v     Principal point coordinates in pixels from calibration
  //  points3           Input 3D points as 3xN matrix (N>=6)
  //  points2           Input 2D points as 2xN matrix (in corresponding order to 3D points)
  //  ransac_threshold  Inlier threshold in terms of reprojection error in pixels
  EpnpRansacAdaptor(const Matrix3Xd& points3, const Matrix2Xd& points2, double focal_u,
                    double focal_v, double principal_u, double principal_v,
                    double ransac_threshold);

  // Check the input points and parameters.
  bool checkInput() const;

  // Accessor for input size.
  unsigned inputSize() const { return points3_.cols(); }

  // Generate a single pose hypothesis from a seed by using the EPnP algorithm, then
  // score the hypothesis using the following soft-scoring scheme.
  // Only inlier 2D-3D matches contribute to the score, namely, by exp(-e^2/(2*sigma^2))
  // where e is the reprojection error norm (in pixels) and sigma = ransac_threshold / 3 (pixels).
  // A match is an inlier if its reprojection error e <= ransac_threshold.
  // If reprojection error e = 0                     -> contribution is 1.0
  // If reprojection error e = 0.33*ransac_threshold -> contribution is 0.607
  // If reprojection error e = 0.66*ransac_threshold -> contribution is 0.135
  // If reprojection error e > ransac_threshold      -> contribution is 0
  // seed: list of indices of a small subset of the input 2D/3D points to generate the hypothesis.
  // These are columns indices of matrices points3 and points2 passed to the constructor.
  ResultType evaluate(const std::array<unsigned, kSampleSize>& seed) const;

  // Decide if two poses (a and b) are similar within tolerance parameters:
  // max_distance       Maximum relative distance between the two camera positions.
  // max_angle_degrees  Maximum angular component of the difference in camera orentations.
  static bool isSimilar(const ModelType& model1, const ModelType& model2,
                        double max_distance = 1e-3, double max_angle_degrees = 1e-3);

  // Compute pose from all inliers.
  // inliers is a list of indices of inliers within the original input 2D-3D point matches.
  // These are columns indices of matrices points3 and points2 passed to the constructor.
  bool refit(const std::vector<unsigned>& inliers, ModelType* pose) const;

 private:
  // Intrinsic parameters and references to input points, assuming the adaptor is short-lived.
  Matrix3d calib_matrix_;
  const Matrix3Xd& points3_;
  const Matrix2Xd& points2_;

  // Coefficients derived from the RANSAC threshold and used for the scoring describe above.
  double score_coeff_;
  double threshold_squared_;
};

}  // namespace epnp
}  // namespace pnp
}  // namespace isaac
