/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "packages/pnp/gems/epnp/epnp.hpp"
#include "packages/pnp/gems/tests/simu.hpp"

// Print command-line usage info.
void usage(void) {
  std::cout << "Usage: epnp_eval runtime" << std::endl;
  std::cout << "       epnp_eval accuracy" << std::endl;
  exit(EXIT_FAILURE);
}

// Minimalistic utility class for easy high-resolution time measurement.
class StopWatch {
 private:
  std::chrono::steady_clock::time_point reset_time;  // Timestamp of last reset() or ctor call.
  std::chrono::steady_clock::time_point split_time;  // Tmiestamp of last split() call.

 public:
  StopWatch() { reset_time = split_time = std::chrono::steady_clock::now(); }

  // Record reset and split timestamps.
  // split() can be called several times after a reset()
  void reset() { reset_time = std::chrono::steady_clock::now(); }
  void split() { split_time = std::chrono::steady_clock::now(); }

  // Access ellapsed time between last reset() or ctor and last split() call.
  double nanoseconds() const {
    return (std::chrono::duration<double, std::nano>(split_time - reset_time).count());
  }
  double microseconds() const { return (1e-3 * nanoseconds()); }
  double milliseconds() const { return (1e-6 * nanoseconds()); }
  double seconds() const { return (1e-9 * nanoseconds()); }
  double minutes() const { return (1e-9 / 60.0 * nanoseconds()); }
};

// Average elements of a std::vector.
double Mean(const std::vector<double>& values) {
  if (values.size()) {
    double sum = 0.0;
    for (double x : values) {
      sum += x;
    }
    return sum / values.size();
  } else {
    return 0.0;
  }
}

// Average elements of an isaac::VectorXd.
double Mean(const isaac::VectorXd& values) {
  if (values.size()) {
    return values.sum() / values.size();
  } else {
    return 0.0;
  }
}

// Compute standard deviation of all elements in a std::vector.
double StdDev(const std::vector<double>& values) {
  if (values.size() > 1) {
    double m = Mean(values);
    double sum = 0.0;
    for (double x : values) {
      sum += (x - m) * (x - m);
    }
    return std::sqrt(sum / (values.size() - 1));
  } else {
    return 0.0;
  }
}

// Compute standard deviation of all elements in an isaac::VectorXd.
double StdDev(const isaac::VectorXd& values) {
  if (values.size() > 1)
    return std::sqrt((values.array() - Mean(values)).matrix().squaredNorm() / (values.size() - 1));
  else
    return 0.0;
}

// Find the element of a vector such that a given ratio of elements fall below its value.
// ratio = 0.7 means 70% of the elements of the vector are within the returned value,
// ratio = 1.0 is equivalent to the maximum of all values,
// ratio = 0.0 is equivalent to the minimum of all values.
double Quantile(const std::vector<double>& vec, double ratio) {
  if (vec.size() == 0) {
    return 0;
  }

  // Clamp the ratio into the range [0,1].
  if (ratio > 1.0) {
    ratio = 1.0;
  } else if (ratio < 0.0) {
    ratio = 0.0;
  }

  // Copy and sort the elements of the vector. Only sorts as much as necessary (partial sort).
  // Map the ratio to an element index into the sorted vector and return the pointed value.
  std::vector<double> v = vec;
  double idx = std::round(ratio * (v.size() - 1));
  auto element = v.begin() + static_cast<size_t>(idx);
  std::nth_element(v.begin(), element, v.end());
  return *element;
}

// ConvertToStdVector between isaac::VectorXd and std::vector.
std::vector<double> ConvertToStdVector(const isaac::VectorXd& vec) {
  std::vector<double> v(vec.size());
  for (int i = 0; i < vec.size(); i++) {
    v[i] = vec(i);
  }
  return v;
}

// Interface for generating 2D/3D point matches for PnP testing.
// This abstraction is useful to hide that different generators have different parameter sets.
class PointGenerator {
 public:
  virtual ~PointGenerator() {}
  // Generate a number of 3D points within the camera field of view and their 2D projections.
  virtual void GeneratePoints(unsigned num_points, const isaac::pnp::Camera& camera,
                              isaac::Matrix3Xd* points3, isaac::Matrix2Xd* points2) const = 0;
  // Print the parameters of the point generator.
  virtual void PrintParams(std::ostream& os) const = 0;
};

// Generator for 2D/3D point matches for PnP testing.
// In short, this is a wrapper for GenerateFovPoints() that hides its special parameters.
// Generates a number of random perfect 2D/3D point matches with respect to a given camera
// such that 3D points are between near and far planes.
class SpatialPointGenerator : public PointGenerator {
 private:
  // Only a wrapper for isaac::pnp::GenerateFovPoints().
  // For the exact meaning of these parameters, see isaac::pnp::GenerateFovPoints().
  double near_;
  double far_;

 public:
  SpatialPointGenerator(double near, double far) : near_(near), far_(far) {}
  // Generate a number of 3D points within the camera field of view and their 2D projections.
  void GeneratePoints(unsigned num_points, const isaac::pnp::Camera& camera,
                      isaac::Matrix3Xd* points3, isaac::Matrix2Xd* points2) const override {
    isaac::pnp::GenerateFovPoints(num_points, camera, near_, far_, points3, points2);
  }
  // Print point generation parameters.
  void PrintParams(std::ostream& os) const override { os << "near " << near_ << " far " << far_; }
};

// Generator for 2D/3D point matches for PnP testing.
// In short, this is a wrapper for GenerateFovPointsPlanar() that hides its special parameters.
// Generates a number of random perfect 2D/3D point matches with respect to a given camera
// such that 3D points are on a random plane but at fixed depth and inclination angle
// with respect to the camera.
class PlanarPointGenerator : public PointGenerator {
 private:
  // Only a wrapper for isaac::pnp::GenerateFovPointsPlanar().
  // For the exact meaning of these parameters, see isaac::pnp::GenerateFovPointsPlanar().
  double depth_;      // Plane depth along the optical axis.
  double angle_;      // Plane inclination angle with respect to the image plane.
  double deviation_;  // Noise standard deviation parameter to move points off-plane.

 public:
  PlanarPointGenerator(double depth, double angle, double deviation)
      : depth_(depth), angle_(angle), deviation_(deviation) {}
  // Generate a number of 2D/3D point matches with respect to a given camera.
  void GeneratePoints(unsigned num_points, const isaac::pnp::Camera& camera,
                      isaac::Matrix3Xd* points3, isaac::Matrix2Xd* points2) const override {
    isaac::Vector4d plane;
    isaac::pnp::GenerateFovPointsPlanar(num_points, camera, depth_, angle_, deviation_, points3,
                                        points2, &plane);
  }
  // Print point generation parameters.
  void PrintParams(std::ostream& os) const override {
    os << "plane_depth " << depth_;
    os << " plane_angle " << angle_;
    os << " plane_deviation " << deviation_;
  }
};

// Debug utility that prints various statistics of a set of generated 2D/3D point matches.
void DumpData(const isaac::pnp::Camera& camera, const isaac::Matrix3Xd& points3,
              const isaac::Matrix2Xd& points2) {
  std::cout << "Simulated data --------------------------- " << std::endl;

  // Print the number of points generated.
  std::cout << "points: " << points3.cols() << std::endl;

  // Measure point depths and print the depth to the closest and farthest point.
  isaac::VectorXd depths = isaac::pnp::ComputeDepths(points3, camera);
  std::cout << "depths: " << depths.minCoeff() << " to " << depths.maxCoeff() << std::endl;

  // Print the size of the bounding boxes around 2D/3D points to see there is a variation in coords.
  std::cout << "bbox 3d: ";
  std::cout << (points3.rowwise().maxCoeff() - points3.rowwise().minCoeff()).transpose();
  std::cout << std::endl;
  std::cout << "bbox 2d: ";
  std::cout << (points2.rowwise().maxCoeff() - points2.rowwise().minCoeff()).transpose();
  std::cout << std::endl;

  // Calculate reprojection errors for the input points.
  isaac::Matrix2Xd proj2 = ProjectPoints(camera, points3, false, false);
  isaac::VectorXd repr_error_norms = isaac::pnp::ColwiseNorms(points2 - proj2);

  // Print reprojection error statistics.
  std::cout << "input repr.err.norms:   ";
  std::cout << "mean " << Mean(repr_error_norms) << "   ";
  std::cout << "max " << repr_error_norms.maxCoeff() << std::endl;
}

// Debug utility that prints EPnP output in detail.
void DumpResult(const isaac::pnp::epnp::Result& result, const isaac::pnp::Camera& camera,
                const isaac::Matrix3Xd& points3, const isaac::Matrix2Xd& points2) {
  std::cout << "EPnP result --------------------------- " << std::endl;

  // Print detected problem dimensionality
  std::cout << "dims: " << result.input_dims << std::endl;

  // Print number of returned control points.
  std::cout << "ctl_points: " << result.ctl_points_world.cols() << std::endl;

  // Print the axis lengths of the chosen 3D basis.
  // This is the distance between the first control point and each subsequent control point.
  std::cout << "ctl_point lengths: ";
  if (result.ctl_points_world.cols() > 1) {
    std::cout << (result.ctl_points_world.col(1) - result.ctl_points_world.col(0)).norm() << " ";
  }
  if (result.ctl_points_world.cols() > 2) {
    std::cout << (result.ctl_points_world.col(2) - result.ctl_points_world.col(0)).norm() << " ";
  }
  if (result.ctl_points_world.cols() > 3) {
    std::cout << (result.ctl_points_world.col(3) - result.ctl_points_world.col(0)).norm() << " ";
  }
  std::cout << std::endl;

  // Calculate and print the depth of each control point.
  isaac::VectorXd ctl_point_depths = isaac::pnp::ComputeDepths(result.ctl_points_world, camera);
  std::cout << "ctl_point depths: " << ctl_point_depths.transpose();

  // Print control point depth statistics.
  std::cout << " (mean " << Mean(ctl_point_depths) << ", delta "
            << ctl_point_depths.maxCoeff() - ctl_point_depths.minCoeff() << ")" << std::endl;

  // Print the size of the matrix of barycentric coordinates.
  std::cout << "barycentric coeffs: ";
  std::cout << result.bary_coeffs.rows() << "x" << result.bary_coeffs.cols() << std::endl;

  // Print maximum algebraic residual for the computed baricentric coefficients.
  // This should always be close to zero.
  std::cout << "barycentric error:  ";
  std::cout << (isaac::pnp::HomogeneousFromEuclidean(result.ctl_points_world) * result.bary_coeffs -
                isaac::pnp::HomogeneousFromEuclidean(points3))
                   .cwiseAbs()
                   .maxCoeff()
            << std::endl;

  // Print singular values of the projection coefficient matrix in increasing order.
  std::cout << "sing.values (" << result.singular_values.size() << "):";
  std::cout << result.singular_values.transpose() << std::endl;

  // Print the ratio between each singular value and the largest singular value.
  std::cout << "normalized:  ";
  std::cout << result.singular_values.transpose() / result.singular_values.maxCoeff() << std::endl;

  // Print the algebraic residuals obtained when substituting the singular vectors corresponding
  // to the least singular values back into the projection equations.
  // Shows how well each base vector of the solution space of the projection equation
  // satisfies the equation itself.
  std::cout << "ctl_point algebraic residuals (singular_values): ";
  for (int k = 0; k < result.solution_basis.cols(); k++) {
    std::cout << (result.proj_coeffs * result.solution_basis.col(k)).norm();
    std::cout << " (" << result.singular_values(k) << ")   ";
  }
  std::cout << std::endl;

  // Consider only the first base vector of solution space if the projection equations as solution.
  // This corresponds to a reconstruction of the control points in camera coordinates.
  // Backproject this solution to the image and print the minimum and maximum reprojection error.
  std::cout << "ctl_point geometric residuals (pixels): ";
  isaac::Matrix3Xd solution = isaac::pnp::epnp::ReshapeToMatrix3xN(result.solution_basis.col(0));
  isaac::VectorXd repr_errors =
      isaac::pnp::ColwiseNorms(points2 - isaac::pnp::epnp::ProjectPoints(
                                             camera.calib_matrix, solution * result.bary_coeffs));
  std::cout << " mean " << Mean(repr_errors) << "  max " << repr_errors.maxCoeff() << std::endl;

  // Calculate the ground-truth control points in the camera frame by transforming the computed
  // control point world coordinates into camera coordinates via the ground-truth pose.
  isaac::Matrix3Xd true_ctl_points_cam =
      camera.rotation_matrix * (result.ctl_points_world.colwise() - camera.position);

  // Compare the control point camera coordinates to the ground-truth coordinates.
  std::cout << "ctl_points vs ground-truth: ";
  std::cout << isaac::pnp::ColwiseNorms(result.ctl_points_cam - true_ctl_points_cam).transpose();
  std::cout << std::endl;

  // Print the reprojection error residual sum-of-squares (RSS) for each hypothesized
  // dimensionality of the solution space (1, 2, 3...)
  std::cout << "residuals per sol.dim: " << result.rss_repr_errors.transpose() << std::endl;

  // Print the dimensionality of the solution space that led to the least reprojection error.
  std::cout << "sol.dims: " << result.solution_dims << std::endl;

  // The computed rotation matrix should be orthonormal and have determinant +1.
  // Print its deviation from orthonormality and its determinant.
  std::cout << "R ortho. error: ";
  std::cout << (result.rotation.transpose() * result.rotation - isaac::Matrix3d::Identity()).norm();
  std::cout << std::endl;
  std::cout << "R determinant:  " << result.rotation.determinant() << std::endl;

  // The computed pose is obtained by 3D registration between the control points in world
  // in world coordinates and control points in camera coordinates.
  // Print the 3D registration residuals to see how good the registration was.
  // Registration should be perfect in the case of noise-free input.
  std::cout << "reg. residuals: ";
  std::cout << isaac::pnp::ColwiseNorms(
                   (result.rotation * result.ctl_points_world - result.ctl_points_cam).colwise() +
                   result.translation)
                   .transpose();
  std::cout << std::endl;

  // Calculate ground-truth translational component from the ground-truth camera position.
  isaac::Vector3d true_translation = -camera.rotation_matrix * camera.position;

  // Calculate camera position from the translational component of the computed pose.
  isaac::Vector3d camera_position = -result.rotation.transpose() * result.translation;

  // Calculate orientation error with respect to ground-truth.
  isaac::Vector3d error_angle_axis =
      isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix.transpose() * result.rotation);

  // Test conversions between angle axis and rotation matrix and
  // make sure the rotation error transforms the computed rotation matrix to the ground-truth.
  // Print residual of this transformation. It should always be zero to numerical precision.
  std::cout << "rodrigues: ";
  std::cout << (camera.rotation_matrix * isaac::pnp::MatrixFromAngleAxis(error_angle_axis) -
                result.rotation)
                   .norm();
  std::cout << std::endl;

  // Print rotation error in degrees, translation error and error in camera position,
  // all with respect to the ground-truth.
  std::cout << "pose error:    ";
  std::cout << "rotation: " << isaac::RadToDeg(error_angle_axis.norm()) << " (deg)   ";
  std::cout << "translation: " << (true_translation - result.translation).norm() << "   ";
  std::cout << "position: " << (camera_position - camera.position).norm() << "   ";
  std::cout << std::endl;
}

// Debug utility that runs EPnP camera pose estimation on different independent random
// synthetic datasets, stops when a certain condition is met and dumps the generated data
// and detailed results for that run.
void DetectAndDump() {
  // Test parameters
  const unsigned num_runs = 1000;  // Number of indpeendent runs to execute.
  unsigned num_failed_runs = 0;    // Number of runs that failed.
  const unsigned num_points = 6;   // Number of 2D/3D point matches to generate in each run.
  const double near = 2.0;         // Depth of near plane for point generation.
  const double far = 6.0;          // Depth of far plane for point generation.
  // const double depth = 5.0;        // Depth of the plane for planar point generation.
  // const double angle = 60.0;       // Inclination angle of the plane for planar point generation.
  const double sigma = 0.0;  // Sigma of the Gaussian noise to add to the ideal image points.

  // Perform many runs on independent datasets.
  for (unsigned i = 0; i < num_runs; i++) {
    // Generate a camera with fixed intrinsics and random pose.
    const int width = 1280;
    const int height = 720;
    const double focal = 700.0;
    isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
    const double focal_u = camera.calib_matrix(0, 0);
    const double focal_v = camera.calib_matrix(1, 1);
    const double principal_u = camera.calib_matrix(0, 2);
    const double principal_v = camera.calib_matrix(1, 2);

    // Generate a number of ideal 2D/3D point matches for this camera.
    isaac::Matrix3Xd points3;
    isaac::Matrix2Xd points2_ideal;
    isaac::Vector4d plane;
    // GenerateFovPointsPlanar(num_points, camera, depth, angle, 0, points3, points2_ideal, plane);
    GenerateFovPoints(num_points, camera, near, far, &points3, &points2_ideal);

    // Add Gaussian noise to the image points (if sigma > 0).
    isaac::Matrix2Xd points2;
    if (sigma) {
      points2 =
          points2_ideal + sigma * isaac::pnp::RandomGaussianMatrix<isaac::Matrix2Xd>(2, num_points);
    } else {
      points2 = points2_ideal;
    }

    // Run the EPnP algorithm with exact intrinsics and test stop condition for dumping info.
    isaac::pnp::epnp::Result result;
    if (isaac::pnp::epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v, points3,
                                            points2, &result) == isaac::pnp::Status::kSuccess) {
      // Construct a camera with ideal intrinsics but estimated pose.
      isaac::pnp::Camera estimated_camera{camera.width, camera.height, camera.calib_matrix,
                                          result.rotation,
                                          -result.rotation.transpose() * result.translation};

      // Calculate reprojection errors using the estimated camera.
      isaac::Matrix2Xd proj_points = ProjectPoints(estimated_camera, points3, false, false);
      isaac::VectorXd repr_errors = isaac::pnp::ColwiseNorms(points2 - proj_points);

      // Compute the error in camera position and rotation.
      isaac::Vector3d camera_position = -result.rotation.transpose() * result.translation;
      double rotation_error = isaac::RadToDeg(
          isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix.transpose() * result.rotation)
              .norm());
      double position_error = (camera_position - camera.position).norm();

      // Check stopping error conditions as needed.
      // if (repr_errors.maxCoeff() > 0.01 || rotation_error > 0.1 || translation_error > 0.01)
      {
        // Print errors to provide some info on the stop condition.
        std::cout << "Debugging i=" << i << "-----------------" << std::endl;
        std::cout << "rotation_error=" << rotation_error;
        std::cout << " position_error=" << position_error << std::endl;
        std::cout << "repr_errors: " << repr_errors.transpose() << " px" << std::endl;

        // Print noise parameter and the estimated standard deviation
        // of the realized additive noise in 2D point locations.
        isaac::Matrix2Xd noise = points2 - points2_ideal;
        std::cout << "noise:   sigma: " << sigma << " px   std: ";
        std::cout << StdDev(Eigen::Map<isaac::VectorXd>(noise.data(), noise.size()));
        std::cout << " px" << std::endl;

        std::cout << std::endl;

        // Dump statistics of the generated data.
        DumpData(camera, points3, points2);
        std::cout << std::endl;

        // Dump detailed info on intermediate and final results.
        DumpResult(result, camera, points3, points2);

        return;
      }
    } else {
      // EPnP returned a failure code. Count such runs.
      num_failed_runs++;
    }
  }
  // Report the number of executed and failed runs.
  std::cout << num_runs << " runs executed, " << num_failed_runs << " failed" << std::endl;
}

// Analysis tool that runs EPnP many times for each focal length value and averages the singular
// values of the projection coefficient matrix for a non-planar configuration.
// This produces data to plot something like Fig.3 in the EPnP paper.
void EvaluateSingularValues(int num_points, int num_runs, double sigma) {
  // Camera intrinsics.
  const int width = 640;
  const int height = 480;
  std::vector<double> focal_lengths{100, 500, 1000, 5000, 10000};

  // Depth of near and far planes to restrict the depth range of generated 3D points.
  const double near = 1.0;
  const double far = 10.0;

  // Print parameters used for this experiment (should be easy to parse).
  std::cout << "resolution " << width << " " << height << std::endl;
  std::cout << "focal";
  for (auto focal : focal_lengths) {
    std::cout << " " << focal;
  }
  std::cout << std::endl;
  std::cout << "depth_range " << near << " " << far << std::endl;
  std::cout << "num_points " << num_points << std::endl;
  std::cout << "sigma " << sigma << std::endl;
  std::cout << "num_runs " << num_runs << std::endl;

  // Run experiments with each of the focal lengths.
  for (auto focal : focal_lengths) {
    // Singular values averaged across all runs for this focal length.
    isaac::VectorXd singular_values = isaac::VectorXd::Zero(12);
    // Count the number of successful runs for averaging.
    unsigned num_successful_runs = 0;

    // Multiple runs with fixed focal length.
    // In each run, generate random input, estimate camera pose from it,
    // and average the singular values.
    for (int i = 0; i < num_runs; i++) {
      // Generate a camera with fixed intrinsics and random pose.
      isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
      const double focal_u = camera.calib_matrix(0, 0);
      const double focal_v = camera.calib_matrix(1, 1);
      const double principal_u = camera.calib_matrix(0, 2);
      const double principal_v = camera.calib_matrix(1, 2);

      // Generate a number of ideal 2D/3D point matches for this camera.
      isaac::Matrix3Xd points3;
      isaac::Matrix2Xd points2;
      isaac::pnp::GenerateFovPoints(num_points, camera, near, far, &points3, &points2);

      // Add noise to the 2D point locations (if sigma > 0).
      if (sigma) {
        isaac::Matrix2Xd noise =
            sigma * isaac::pnp::RandomGaussianMatrix<isaac::Matrix2Xd>(2, num_points);
        points2 += noise;
      }

      // Run the EPnP algorithm with exact intrinsics.
      isaac::pnp::epnp::Result result;
      if (isaac::pnp::epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v, points3,
                                              points2, &result) == isaac::pnp::Status::kSuccess) {
        // Cumulate singular values.
        singular_values += result.singular_values;
        num_successful_runs++;
      }
    }
    // Calculate average singular values.
    singular_values /= static_cast<double>(num_successful_runs);

    // Print focal length and the averaged singular values of the projection equation.
    std::cout << focal << " " << singular_values.transpose() << std::endl;
  }
}

// Benchmark EPnP runtime for several input sizes (number of 2D/3D matches)
// by averaging over many independent random synthetic datasets and many runs for each dataset.
void RuntimeBenchmark(int num_datasets = 100, int num_runs = 10) {
  // Print header and the parameters of this benchmark.
  std::cout << "EPnP (CPU version) timing benchmark" << std::endl;
  std::cout << "Averaging over " << num_datasets;
  std::cout << " random datasets (non-planar, noise-free) and ";
  std::cout << num_runs << " runs per dataset" << std::endl;

  // Statistics for time averaging.
  int num_total_runs = 0;       // Number of total calls to EPnP.
  int num_successful_runs = 0;  // Number of successful calls to EPnP.

  // Make sure printed lines are aligned.
  std::cout << std::fixed << std::setprecision(3);

  // Repeat experiments for different numbers of 2D/3D point matches generated.
  std::vector<unsigned> nums_points{6, 8, 10, 20, 50, 100, 200, 500, 1000};
  StopWatch timer;
  for (auto num_points : nums_points) {
    // Print the current number of points being tested.
    std::cout << std::setw(4) << num_points << " points... ";
    std::cout.flush();

    // Run pose estimation on many independent datasets, each with the same number of 2D/3D matches.
    std::vector<double> runtimes;
    for (int i = 0; i < num_datasets; i++) {
      // Generate a camera with fixed intrinsics and random pose.
      isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(1280, 720, 700.0);
      const double focal_u = camera.calib_matrix(0, 0);
      const double focal_v = camera.calib_matrix(1, 1);
      const double principal_u = camera.calib_matrix(0, 2);
      const double principal_v = camera.calib_matrix(1, 2);

      // Generate a number of ideal 2D/3D point matches for this camera.
      isaac::Matrix3Xd points3;
      isaac::Matrix2Xd points2;
      isaac::pnp::GenerateFovPoints(num_points, camera, 1, 10, &points3, &points2);

      // Run pose estimation on the current dataset many times.
      timer.reset();
      for (int j = 0; j < num_runs; j++) {
        num_total_runs++;
        isaac::pnp::epnp::Result result;
        if (isaac::pnp::epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v, points3,
                                                points2, &result) == isaac::pnp::Status::kSuccess)
          num_successful_runs = num_successful_runs + 1;
      }
      // Record the average runtime over these runs.
      timer.split();
      runtimes.push_back(timer.milliseconds() / num_runs);
    }

    // Calculate the average runtime over the random datasets, as well as the standard deviation.
    std::cout << Mean(runtimes) << " ms (std: " << StdDev(runtimes) << ")" << std::endl;
  }

  // Report the number of total and successful runs.
  std::cout << num_successful_runs << " of ";
  std::cout << num_total_runs << " runs reported success" << std::endl;
}

// Utility function to output a std::stringstream to two output streams.
// This is used to write the same text to both the terminal and into a file.
void Print(std::stringstream& ss, std::ostream& os1, std::ostream& os2, bool clear = true) {
  if (os1.good()) {
    os1 << ss.str();
  }
  if (os2.good()) {
    os2 << ss.str();
  }
  if (clear) {
    ss.clear();
    ss.str("");
  }
}

// Benchmark EPnP pose accuracy for several input sizes (number of 2D/3D matches) and
// image noise levels by collecting error statistics from many independent random
// synthetic datasets. Camera intrinsics are assumed to be known exactly in this benchmark.
void AccuracyBenchmark(int num_runs, int width, int height, double focal,
                       const PointGenerator& point_generator, const char* filename = 0) {
  // Number of points and noise levels to benchmark.
  // Noise levels are Gaussian sigmas of the noise added to 2D point locations.
  std::vector<unsigned> nums_points{6, 8, 12, 20, 35, 50, 100, 200, 1000};
  std::vector<double> sigmas{0, 0.1, 1, 2, 3, 4, 5};

  // Prepare the output streams.
  // If filename given, the benchmark will produce the same output to the terminal and to the file.
  std::ofstream file;
  if (filename) {
    file.open(filename);
  }

  // Print benchmark type and benchmark parameters in a format that is easy to parse.
  std::stringstream ss;
  ss << "epnp_benchmark accuracy " << std::endl;
  ss << "runs " << num_runs << " ";
  ss << "focal " << focal << " ";
  ss << "width " << width << " ";
  ss << "height " << height << " ";
  point_generator.PrintParams(ss);
  ss << std::endl << std::endl;
  Print(ss, std::cout, file);

  // Print parameter names in the column headers.
  const int column_width = 8;
  ss << std::setw(column_width) << "pts"
     << " ";
  ss << std::setw(column_width) << "noise"
     << " ";
  ss << std::setw(column_width) << "n.std"
     << " ";
  ss << std::setw(column_width) << "n.max"
     << " ";
  ss << std::setw(column_width) << "rep.max"
     << " ";
  ss << std::setw(column_width) << "rep.max"
     << " ";
  ss << std::setw(column_width) << "rep.90%"
     << " ";
  ss << std::setw(column_width) << "rep.90%"
     << " ";
  ss << std::setw(column_width) << "rep>5x"
     << " ";
  ss << std::setw(column_width) << "rot"
     << " ";
  ss << std::setw(column_width) << "rot"
     << " ";
  ss << std::setw(column_width) << "pos"
     << " ";
  ss << std::setw(column_width) << "pos"
     << " ";
  ss << std::setw(column_width) << "runs"
     << " ";
  ss << std::endl;
  // Print jointly to terminal and file but this line will be a comment in the file.
  file << "#";
  Print(ss, std::cout, file);

  // Print the type of values printed in each column.
  //  param   Each value is a parameter.
  //  mean    Each value is an average.
  //  90%     Each value is an upper bound on 90% of some raw data.
  ss << std::setw(column_width) << "param"
     << " ";
  ss << std::setw(column_width) << "param"
     << " ";
  ss << std::setw(column_width) << "mean"
     << " ";
  ss << std::setw(column_width) << "mean"
     << " ";
  ss << std::setw(column_width) << "mean"
     << " ";
  ss << std::setw(column_width) << "90%"
     << " ";
  ss << std::setw(column_width) << "mean"
     << " ";
  ss << std::setw(column_width) << "90%"
     << " ";
  ss << std::setw(column_width) << "mean"
     << " ";
  ss << std::setw(column_width) << "mean"
     << " ";
  ss << std::setw(column_width) << "90%"
     << " ";
  ss << std::setw(column_width) << "mean"
     << " ";
  ss << std::setw(column_width) << "90%"
     << " ";
  ss << std::setw(column_width) << ""
     << " ";
  ss << std::endl;
  // Print jointly to terminal and file but this line will be a comment in the file.
  file << "#";
  Print(ss, std::cout, file);

  // Print the unit of values in each column.
  //  3d    Distance in 3D.
  //  deg   Angular error in degrees.
  //  %     Percentage.
  //  #     Simple count (no unit)..
  ss << std::setw(column_width) << "#"
     << " ";
  ss << std::setw(column_width) << "px"
     << " ";
  ss << std::setw(column_width) << "px"
     << " ";
  ss << std::setw(column_width) << "px"
     << " ";
  ss << std::setw(column_width) << "px"
     << " ";
  ss << std::setw(column_width) << "px"
     << " ";
  ss << std::setw(column_width) << "px"
     << " ";
  ss << std::setw(column_width) << "px"
     << " ";
  ss << std::setw(column_width) << "%"
     << " ";
  ss << std::setw(column_width) << "deg"
     << " ";
  ss << std::setw(column_width) << "deg"
     << " ";
  ss << std::setw(column_width) << "3d"
     << " ";
  ss << std::setw(column_width) << "3d"
     << " ";
  ss << std::setw(column_width) << "#"
     << " ";
  ss << std::endl;
  ss << std::endl;
  // Print jointly to terminal and file but this line will be a comment in the file.
  file << "#";
  Print(ss, std::cout, file);

  // Benchmark EPnP with different input sizes.
  for (unsigned num_points : nums_points) {
    // Benchmark EPnP with different noise levels.
    for (double sigma : sigmas) {
      // Performance measures to collect over many runs.
      std::vector<double> repr_errors_90;    // Repr. error norm upper bound for 90% of points.
      std::vector<double> mean_repr_errors;  // Average of reprojection error norms in each run.
      std::vector<double> max_repr_errors;   // Largest of reprojection error norms in each run.
      std::vector<double> rot_errors;        // Camera rotational error (in degrees) in each run.
      std::vector<double> pos_errors;        // Camera position error in each run.
      std::vector<double> noise_std;       // Std deviation of the realized image coordinate noise.
      std::vector<double> noise_max_norm;  // Maximum norm of the realized image coordinate noise.
      unsigned num_runs_large_errors = 0;  // Count runs when max. repr. error exceeds a threshold.
      unsigned num_successful_runs = 0;    // Count runs when PnP function reports success.

      // Threshold in pixels for considering the reprojection error for a 2D/3D point match
      // exceptionally large relative to the noise level.
      const double threshold = 5.0 * sigma;

      // Several runs with independent random synthetic datasets.
      for (int i = 0; i < num_runs; i++) {
        // Generate a camera with fixed intrinsics and random pose.
        isaac::pnp::Camera camera = isaac::pnp::GenerateRandomCamera(width, height, focal);
        const double focal_u = camera.calib_matrix(0, 0);
        const double focal_v = camera.calib_matrix(1, 1);
        const double principal_u = camera.calib_matrix(0, 2);
        const double principal_v = camera.calib_matrix(1, 2);

        // Generate a number of ideal 2D/3D point matches in front of the generated camera.
        isaac::Matrix3Xd points3;
        isaac::Matrix2Xd points2_ideal;
        point_generator.GeneratePoints(num_points, camera, &points3, &points2_ideal);

        // Add noise to the location of the 2D points in the image.
        isaac::Matrix2Xd points2;
        if (sigma) {
          points2 = points2_ideal +
                    sigma * isaac::pnp::RandomGaussianMatrix<isaac::Matrix2Xd>(2, num_points);
        } else {
          points2 = points2_ideal;
        }

        // Collect noise statistics: standard deviation and the norm of the largest displacement.
        isaac::Matrix2Xd noise = points2 - points2_ideal;
        noise_max_norm.push_back(isaac::pnp::ColwiseNorms(noise).maxCoeff());
        noise_std.push_back(StdDev(Eigen::Map<isaac::VectorXd>(noise.data(), noise.size())));

        // Run EPnP algorithm
        isaac::pnp::epnp::Result result;
        if (isaac::pnp::epnp::ComputeCameraPose(focal_u, focal_v, principal_u, principal_v, points3,
                                                points2, &result) == isaac::pnp::Status::kSuccess) {
          // Construct a camera with ideal intrinsics but estimated pose.
          isaac::pnp::Camera estimated_camera{camera.width, camera.height, camera.calib_matrix,
                                              result.rotation,
                                              -result.rotation.transpose() * result.translation};

          // Calculate reprojection errors using the estimated camera.
          isaac::Matrix2Xd proj_points = ProjectPoints(estimated_camera, points3, false, false);
          isaac::VectorXd repr_errors = isaac::pnp::ColwiseNorms(points2 - proj_points);
          double max_repr_error = repr_errors.maxCoeff();

          // Collect reprojection error statistics.
          if (sigma && max_repr_error > threshold) {
            num_runs_large_errors++;
          }
          max_repr_errors.push_back(max_repr_error);
          mean_repr_errors.push_back(Mean(repr_errors));
          repr_errors_90.push_back(Quantile(ConvertToStdVector(repr_errors), 0.9));

          // Compute the error in camera position and rotation.
          isaac::Vector3d camera_position = -result.rotation.transpose() * result.translation;
          double rotation_error = isaac::RadToDeg(
              isaac::pnp::AngleAxisFromMatrix(camera.rotation_matrix.transpose() * result.rotation)
                  .norm());
          double position_error = (camera_position - camera.position).norm();
          rot_errors.push_back(rotation_error);
          pos_errors.push_back(position_error);

          num_successful_runs++;
        }
      }
      // Print various statistics of the performance measures.
      // Produces one additional row of the benchmark table.
      ss << std::fixed << std::setprecision(2);
      ss << " " << std::setw(column_width) << num_points << " ";
      ss << std::setw(column_width) << sigma << " ";
      ss << std::setw(column_width) << Mean(noise_std) << " ";
      ss << std::setw(column_width) << Mean(noise_max_norm) << " ";
      ss << std::setw(column_width) << Mean(max_repr_errors) << " ";
      ss << std::setw(column_width) << Quantile(max_repr_errors, 0.9) << " ";
      ss << std::setw(column_width) << Mean(repr_errors_90) << " ";
      ss << std::setw(column_width) << Quantile(repr_errors_90, 0.9) << " ";
      ss << std::setw(column_width) << 100.0 * num_runs_large_errors / num_runs << " ";
      ss << std::setprecision(4);
      ss << std::setw(column_width) << Mean(rot_errors) << " ";
      ss << std::setw(column_width) << Quantile(rot_errors, 0.9) << " ";
      ss << std::setw(column_width) << Mean(pos_errors) << " ";
      ss << std::setw(column_width) << Quantile(pos_errors, 0.9) << " ";
      ss << std::setw(column_width) << num_successful_runs << " ";
      ss << std::endl;
      // Print the same row to the terminal and into a file.
      Print(ss, std::cout, file);
    }
    std::cout << std::endl;
    file << std::endl;
  }
}

int main(int nargs, const char* args[]) {
  if (nargs != 2) {
    usage();
  }
  std::string method(args[1]);

  if (method == "runtime") {
    RuntimeBenchmark();

  } else if (method == "accuracy") {
    // Run EPnP benchmark on different types of input and save results in text files.

    // Non-planar input and large depth range.
    AccuracyBenchmark(500, 1280, 720, 700.0, SpatialPointGenerator(1.0, 10.0),
                      "epnp-eval-f700-nonplanar.txt");

    // Points on a fronto-parallel plane.
    AccuracyBenchmark(500, 1280, 720, 700.0, PlanarPointGenerator(5.0, 0, 0),
                      "epnp-eval-f700-planar-5m-fronto.txt");

    // Points on a slanted plane.
    AccuracyBenchmark(500, 1280, 720, 700.0, PlanarPointGenerator(5.0, 30.0, 0),
                      "epnp-eval-f700-planar-5m-slanted.txt");

    // Points on a slanted plane with a slight random displacement off the plane.
    AccuracyBenchmark(500, 1280, 720, 700.0, PlanarPointGenerator(5.0, 30.0, 1e-2),
                      "epnp-eval-f700-quasiplanar-5m-slanted.txt");

  } else if (method == "singular_values") {
    // Average singular values of the projection coefficient matrix for many runs
    // and for a number of different focal lengths.
    EvaluateSingularValues(10, 100, 1.0);

  } else if (method == "dump") {
    // Run EPnP many times for independent random datasets,
    // capture special conditions and dump info.
    DetectAndDump();

  } else {
    // Print command-line usage info.
    usage();
  }

  return EXIT_SUCCESS;
}
