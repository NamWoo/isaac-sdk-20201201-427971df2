/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Example camera pose estimation using EPnP (assumes no outliers).

#include <algorithm>
#include <vector>

#include "engine/core/logger.hpp"
#include "packages/pnp/gems/pnp.hpp"

int main() {
  // Example camera intrinsic parameters for a 1280x720 camera.
  // Replace these with the intrinsics of your camera.
  double focal_u = 700.0;
  double focal_v = 700.0;
  double principal_u = 640.0;
  double principal_v = 360.0;

  // Example 3D points and corresponding 2D projections.
  // Respective columns of points2 and points3 are assumed 2D-3D matches under perspective
  // projection. 7 input points are indeed inliers but 3 points are outliers (30% outliers).
  // Replace these points with your 2D-3D matches.
  isaac::Matrix3Xd points3(3, 10);
  isaac::Matrix2Xd points2(2, 10);
  points3 << 20.8431, 22.7066, 12.5419, 24.3453, 23.1531, 17.3267, 23.8903, 23.3591, 22.5806,
      25.1911, -87.0048, -71.5196, -65.4782, -72.8224, -70.6375, -50.1062, -72.3619, -73.7432,
      -71.9111, -73.8643, 74.0062, 54.2797, 33.4676, 55.7185, 56.3711, 25.7721, 60.4177, 53.8373,
      53.7429, 58.1667;
  points2 << 412.35, 297.412, 185.941, 522.347, 727.804, 952.014, 1177.91, 38.1886, 110.989,
      799.057, 206.387, 39.3385, 554.521, 458.627, 88.946, 574.843, 512.34, 554.266, 51.1556,
      672.035;

  // Set the number of RANSAC experiments manually or use the approximative RANSAC formula
  // if you have a larger number of input points. EvaluateRansacFormula() below calculates the
  // number of experiments necessary to succeed with at least 99% probability given 30% inliers.
  unsigned ransac_rounds = std::max(isaac::pnp::EvaluateRansacFormula(0.99, 0.3, 6), 100u);
  LOG_INFO("ransac experiments: %d", ransac_rounds);

  // Example for computing the camera pose using EPnP+RANSAC.
  double ransac_threshold = 1.0;
  unsigned max_top_hypotheses = 1;
  unsigned rand_seed = 73;
  std::vector<isaac::pnp::PoseHypothesis> top_hypotheses = isaac::pnp::ComputeCameraPoseEpnpRansac(
      points3, points2, focal_u, focal_v, principal_u, principal_v, ransac_rounds, ransac_threshold,
      max_top_hypotheses, rand_seed);

  // Use pose hypotheses...
  // For example print them.
  LOG_INFO("%d pose hypotheses found", top_hypotheses.size());
  if (top_hypotheses.size()) {
    for (const auto& hyp : top_hypotheses) {
      std::stringstream ss;
      for (auto index : hyp.inliers) ss << " " << index;
      LOG_INFO("Hypothesis score=%.3f, %d inliers: %s", hyp.score, hyp.inliers.size(),
               ss.str().c_str());
      LOG_INFO("translation=(%.3f %.3f %.3f), axis=(%.3f %.3f %.3f), angle=%.4f",
               hyp.pose.translation(0), hyp.pose.translation(1), hyp.pose.translation(2),
               hyp.pose.rotation.angle(), hyp.pose.rotation.axis()(0), hyp.pose.rotation.axis()(1),
               hyp.pose.rotation.axis()(2), hyp.pose.rotation.angle());

      // Example of calculating the reprojection errors in the image given the estimated pose.
      LOG_INFO("Reprojection errors (pixels):");
      isaac::Matrix3d calib_matrix;
      calib_matrix << focal_u, 0, principal_u, 0, focal_v, principal_v, 0, 0, 1;
      for (unsigned i = 0; i < points3.cols(); i++) {
        isaac::Vector3d proj =
            calib_matrix * (hyp.pose.rotation.matrix() * points3.col(i) + hyp.pose.translation);
        isaac::Vector2d point2(proj(0) / proj(2), proj(1) / proj(2));
        double repr_error = (points2.col(i) - point2).norm();
        LOG_INFO("point %d: %.2f", i, repr_error);
      }
    }
  }

  return 0;
}
