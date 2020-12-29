/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Example camera pose estimation using EPnP (assumes no outliers).

#include "engine/core/logger.hpp"
#include "packages/pnp/gems/pnp.hpp"

int main() {
  // Example camera intrinsic parameters for a 1280x720 camera.
  // Replace these with the intrinsics of your camera.
  double focal_u = 700.0;
  double focal_v = 700.0;
  double principal_u = 640.0;
  double principal_v = 360.0;

  // Example 3D points and corresponding 2D projections (no outliers in this example).
  // Respective columns of points2 and points3 are 2D-3D matches under perspective projection.
  // Replace these with your 2D-3D matches.
  isaac::Matrix3Xd points3(3, 6);
  isaac::Matrix2Xd points2(2, 6);
  points3 << -54.456, -35.9611, -3.74402, -31.7486, -25.3701, -10.4051,
             -130.14, -95.204, -200.175, -94.076, -138.224, -172.544,
             -69.6657, -81.8504, -88.4238, -83.642, -65.8114, -55.4969;
  points2 << 476.962, 364.776, 1053.62, 1126.53, 939.25, 1034.45,
             458.244, 613.827, 178.237, 471.538, 500.726, 478.192;

  // Computing the camera pose using the EPnP algorithm.
  isaac::Pose3d pose;
  if (isaac::pnp::ComputeCameraPoseEpnp(points3, points2, focal_u, focal_v, principal_u,
                                        principal_v, &pose) == isaac::pnp::Status::kSuccess) {
    // Use computed pose ...
    // Below we just print it.
    LOG_INFO("translation=(%.3f %.3f %.3f), axis=(%.3f %.3f %.3f), angle=%.4f",
             pose.translation(0), pose.translation(1), pose.translation(2), pose.rotation.angle(),
             pose.rotation.axis()(0), pose.rotation.axis()(1), pose.rotation.axis()(2),
             pose.rotation.angle());

    // Example of calculating the reprojection errors in the image given the estimated pose.
    LOG_INFO("Reprojection errors (pixels):");
    isaac::Matrix3d calib_matrix;
    calib_matrix << focal_u, 0, principal_u, 0, focal_v, principal_v, 0, 0, 1;
    for (int i = 0; i < points3.cols(); i++) {
      isaac::Vector3d hom_coords =
          calib_matrix * (pose.rotation.matrix() * points3.col(i) + pose.translation);
      isaac::Vector2d point2(hom_coords(0) / hom_coords(2), hom_coords(1) / hom_coords(2));
      LOG_INFO("point %d: %.4f", i, (points2.col(i) - point2).norm());
    }
  }
  return 0;
}
