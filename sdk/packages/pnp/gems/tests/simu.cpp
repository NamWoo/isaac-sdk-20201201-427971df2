/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/pnp/gems/tests/simu.hpp"

#include <vector>

// Seed for random number generation.
constexpr int kRandomSeed = 1352;

namespace isaac {
namespace pnp {

// Single global random engine used in all simulations.
// Same seed and deterministic results between executions.
std::mt19937& GlobalRandomEngine() {
  static std::mt19937 rand_engine{kRandomSeed};
  return rand_engine;
}

// Simulate a perspective camera with fixed intrinsics but random rotation and translation.
Camera GenerateRandomCamera(int width, int height, double focal, double position_range) {
  // Initialize camera intrinsics.
  Camera camera;
  camera.width = width;
  camera.height = height;
  camera.calib_matrix << focal, 0, 0.5 * width, 0, focal, 0.5 * height, 0, 0, 1;

  // Generate uniform random points on the sphere as follows:
  // Repeat generating a random point uniformly within a cube cenetered at the origin
  // until the point falls within the unit sphere inside the cube, then normalize the point.
  Vector3d direction;
  do {
    // Generate a vector of 3 elements, each element a uniform random number in the range [-1,1].
    direction = RandomVector<VectorXd>(3, -1.0, 1.0);
    // Only accept the vector if it is within the unit sphere.
  } while (direction.squaredNorm() > 1.0);
  direction.normalize();

  // Draw a random angle in the range [0, pi].
  std::uniform_real_distribution<double> distribution(0, Pi<double>);
  double angle = distribution(GlobalRandomEngine());

  // Compute camera orientation as a rotation matrix.
  camera.rotation_matrix = MatrixFromAngleAxis(angle * direction);

  // Generate random position with uniform distribution within a cube centered at the origin.
  position_range = std::abs(position_range);
  camera.position = RandomVector<VectorXd>(3, -position_range, position_range);

  return camera;
}

// Generate random 2D points uniformly distributed in the image.
Matrix2Xd GenerateImagePoints(unsigned num_points, unsigned width, unsigned height) {
  Matrix2Xd points(2, num_points);
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  double w = static_cast<double>(width);
  double h = static_cast<double>(height);
  for (unsigned i = 0; i < num_points; i++) {
    points(0, i) = w * distribution(GlobalRandomEngine());
    points(1, i) = h * distribution(GlobalRandomEngine());
  }
  return points;
}

// Generate 3D points within the camera FoV randomly between the near and far planes, as well as
// corresponding 2D points by first. First generate 2D points uniformly in the image and then
// lift them to a random uniform depth between near and far (as measured along the optical axis).
// This approach guarantees that all num_points points are visible within the camera FoV.
void GenerateFovPoints(unsigned num_points, const Camera& camera, double near, double far,
                       Matrix3Xd* points3, Matrix2Xd* points2) {
  if (points2 == nullptr || points3 == nullptr) {
    return;
  }
  points3->resize(3, 0);
  points2->resize(2, 0);

  // The near plane should be in front of the camera's principal plane,
  // and the far plane should be further away.
  if (near <= 0 || far <= near) {
    return;
  }

  // Generate random 2D points in the image.
  *points2 = GenerateImagePoints(num_points, camera.width, camera.height);
  *points3 = HomogeneousFromEuclidean(*points2);

  // Transform 2D points in the image into 3D directions in world coordinates.
  *points3 = camera.rotation_matrix.transpose() * camera.calib_matrix.inverse() * (*points3);

  // Lift 2D points to a random depth in the range [near, far] as measured along the optical axis.
  double depth_range = far - near;
  std::uniform_real_distribution<double> distribution(0, 1.0);
  for (unsigned i = 0; i < num_points; i++) {
    double depth = near + depth_range * distribution(GlobalRandomEngine());
    points3->col(i) *= depth;
    points3->col(i) += camera.position;
  }
}

// Generate random 2D-3D point matches given a perspective view such that all 3D points lie
// within the camera FoV on a plane in front of the camera.
// Method: A random plane is generated at a fixed depth and with a fixed inclination angle
// with respect to the image plane. 2D points are generated uniformly and projected to the plane.
// However, when the plane horizon is in the image (happens at higher inclination angles),
// 2D points above the horizon do not project to the plane in front of the camera.
// Such points are rejected and new points are generated instead and tested until the desired
// amount of points are met.
// The plane is guaranteed to be in front of the camera because it is parameterized by its
// (positive) depth along the optical axis. This guarantees that at least half of the image
// observes the plane, assuming that the principal point is near the image center.
// If the latter is not the case, a slanted plane may be observed in a small portion of the image
// and the chance of a point being rejected increases, which increases the runtime.
// However, such a camera is highly unlikely.
void GenerateFovPointsPlanar(unsigned num_points, const Camera& camera, double depth, double angle,
                             double deviation, Matrix3Xd* points3, Matrix2Xd* points2,
                             Vector4d* plane) {
  if (points2 == nullptr || points3 == nullptr) {
    return;
  }

  points3->resize(3, 0);
  points2->resize(2, 0);

  // Check and clamp input parameters.
  if (depth <= 0) {
    depth = 1e-6;
  }
  if (angle < 0 || angle >= 85) {
    angle = 85;
  }
  if (deviation < 0) {
    deviation = 0;
  }

  // Limit depth at far plane to avoid numerical instabilities.
  const double far = 100.0;

  // Camera looking (optical axis) direction.
  Vector3d camera_dir = camera.rotation_matrix.row(2);
  camera_dir.normalize();  // eliminate numerical errors if any

  // Generate a random plane normal in a given angle with respect to camera_dir.
  Vector3d normal = camera_dir;
  if (angle) {
    // Generate a random vector parallel to the image plane.
    std::uniform_real_distribution<double> uniform(0, 1);
    double x = uniform(GlobalRandomEngine());
    double y = uniform(GlobalRandomEngine());
    const double tol = 1e-5;
    if (x * x + y * y < tol * tol) {  // highly unlikely but make sure to avoid null vector
      x = 1.0;
      y = 0.0;
    }
    Vector3d ortho_vector = x * camera.rotation_matrix.row(0) + y * camera.rotation_matrix.row(1);

    // Generate plane normal that has the prescribed angle with respect to the image plane.
    double tan_angle = std::abs(std::tan(DegToRad(angle)));
    normal = camera_dir + ortho_vector * tan_angle / ortho_vector.norm();
    normal.normalize();
  }

  // Calculate the pivot, the intersection point between the optical axis and the plane.
  Vector3d pivot = camera.position + camera_dir * depth;

  // Determine the plane passing through the pivot with the given normal direction.
  *plane = Vector4d(normal(0), normal(1), normal(2), -normal.dot(pivot));

  // Generate random points, project them to the plane and verify their depth.
  // Points may back-project to the plane behind the camera when the plane horizon is visible.
  // However, the plane is always visible through the principal point -> Guarantee that it
  // covers at least half of the image -> points have max 50% chance to fail
  // so a regeneration method will eventually converge.
  points2->resize(2, num_points);
  points3->resize(3, num_points);

  // Precompute matrix that maps a homogeneous image point to a 3D world direction.
  Matrix3d point_to_ray_matrix = camera.rotation_matrix.transpose() * camera.calib_matrix.inverse();

  // Precompute factor used for point back-projection.
  double factor = depth * normal.dot(camera_dir);

  // Repeat random image point generation, back-projection and depth test.
  unsigned num_valid_points = 0;
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::normal_distribution<double> gaussian(0, 1);
  while (num_valid_points < num_points) {
    // Generate point with uniform distribution within the image.
    double u = camera.width * uniform(GlobalRandomEngine());
    double v = camera.height * uniform(GlobalRandomEngine());

    // Calculate world direction of the ray passing through (u,v).
    Vector3d ray_dir = point_to_ray_matrix * Vector3d(u, v, 1.0);

    // Calculate depth of plane along the ray through (u,v).
    double denominator = normal.dot(ray_dir);
    if (std::abs(denominator) < 1e-6) {  // viewing ray parallel to the plane
      continue;
    }
    double point_depth = factor / denominator;

    // (u,v) projects to the plane behind the camera -> Generate another point.
    // The plane is always visible through the principal point.
    // Therefore, the plane covers at least around half of the image, assuming that the principal
    // point is near the image center. So there is at most around 50% chance of failure.
    if (point_depth <= 0) {
      continue;
    }

    // Limit the depth to avoid numerical issues -> Generate another point if max depth exceeded.
    // This happens to image points close to the horizon (horizon points map to infinity in 3D).
    if (point_depth > far) {
      continue;
    }

    // Otherwise accept the 2D point.
    points2->col(num_valid_points) << u, v;

    // Optionally simulate deviations from planar when deviation is non-zero.
    // Add Gaussian noise only to the point depth (instead of to x,y,z).
    // This guarantees that the 3D point does not move out of the camera FoV.
    if (deviation) {
      // Draw a random noise value from the standard normal distribution (sigma = 1)
      double noise = gaussian(GlobalRandomEngine());

      // Make sure the norm of the displacement does not exceed 3*sigma in order
      // to ease deterministic testing.
      if (noise > 3.0) {
        noise = 3.0;
      } else if (noise < -3.0) {
        noise = -3.0;
      }

      // Scale the depth noise so sigma = deviation for its Gaussian distribution.
      // Divide by norm of ray_dir because the noise is expressed as distance along the ray
      // (unlike point_depth, measured along the optical axis).
      // This guarantees that the maximum distance from the plane is < 3*deviation.
      // This maximum distance may only occur when the plane is fronto-parallel.
      noise *= deviation / ray_dir.norm();

      // Add depth noise to the depth of the point on the plane.
      point_depth += noise;

      // Keep point in front of the camera if the additive noise moved it behind.
      if (point_depth <= 1e-3) {
        point_depth = 1e-3;
      }
    }

    // intersect plane with the ray to receive the corresp. 3d point.
    points3->col(num_valid_points) = camera.position + point_depth * ray_dir;

    num_valid_points++;
  }
}

// generate outlier 2D-3D matches to test robustness for outliers
// 3D points are sampled uniformly in a box around the camera (some visible, some not)
// 2D points are sampled uniformly within the image
std::vector<int> InsertOutliers(unsigned num_outliers, const Camera& cam, double radius,
                                Matrix3Xd* points3, Matrix2Xd* points2, bool shuffle) {
  // List of column indices of the generated outliers in the output points2 and points3.
  std::vector<int> indices;

  // Check input.
  if (points3 == nullptr || points2 == nullptr) {
    return indices;
  }
  if (points3->cols() != points2->cols()) {
    return indices;
  }
  if (num_outliers < 1) {
    return indices;
  }
  if (radius < 0) {
    return indices;
  }

  // Allocate output matrix of (num_points_in + num_outliers) 2D/3D points.
  unsigned num_points_in = points3->cols();
  unsigned num_points_out = num_points_in + num_outliers;
  Matrix3Xd out3(3, num_points_out);
  Matrix2Xd out2(2, num_points_out);

  // Create and initialize indicator vector for input points vs. generated outliers.
  // Each element in the indicator vector corresponds to a column in the two output matrices.
  // Input points are followed by outliers by default.
  std::vector<bool> is_outlier(num_points_out);
  for (unsigned i = 0; i < num_points_in; i++) {
    is_outlier[i] = false;
  }
  for (unsigned i = num_points_in; i < num_points_out; i++) {
    is_outlier[i] = true;
  }

  // Randomly shuffle elements of the indicator vector (the input/outlier order).
  if (shuffle) {
    std::shuffle(is_outlier.begin(), is_outlier.end(), GlobalRandomEngine());
  }

  // Cycle through the input/outlier indicator vector and fill in the output matrices accordingly.
  std::uniform_real_distribution<double> distribution(0, 1);
  int input_index = 0;  // index to the next inlier to copy
  for (unsigned k = 0; k < num_points_out; k++) {
    // Current point should be an outlier.
    if (is_outlier[k]) {
      // Generate random 2D point in the image.
      out2(0, k) = cam.width * distribution(GlobalRandomEngine());
      out2(1, k) = cam.height * distribution(GlobalRandomEngine());

      // Generate random 3D point around the camera independently from the generated 2D point.
      out3(0, k) = radius * (2 * distribution(GlobalRandomEngine()) - 1.0) + cam.position(0);
      out3(1, k) = radius * (2 * distribution(GlobalRandomEngine()) - 1.0) + cam.position(1);
      out3(2, k) = radius * (2 * distribution(GlobalRandomEngine()) - 1.0) + cam.position(2);

      // Record the output column index of the generated outlier.
      indices.push_back(k);

      // Current point should be an input point.
    } else {
      // Copy the next point from the input.
      out3.col(k) = points3->col(input_index);
      out2.col(k) = points2->col(input_index);
      input_index++;
    }
  }

  // Copy generated input + outlier mixture to the output.
  *points3 = out3;
  *points2 = out2;

  // Return the output column index list of the added outliers.
  return indices;
}

}  // namespace pnp
}  // namespace isaac
