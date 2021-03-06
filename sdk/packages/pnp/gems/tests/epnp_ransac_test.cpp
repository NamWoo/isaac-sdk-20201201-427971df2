/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include <iostream>
#include <sstream>
#include <random>

#include "packages/pnp/gems/epnp/epnp_ransac.hpp"
#include "packages/pnp/gems/tests/simu.hpp"
#include "engine/core/logger.hpp"

// Print a RANSAC pose hypothesis.
std::ostream& operator<<(std::ostream& os,
                         const isaac::pnp::epnp::EpnpRansacAdaptor::ResultType& hypothesis) {
  // Print indices of input elements used as sample to generate this hypothesis.
  os << "seed:";
  for (auto idx : hypothesis.seed) {
    os << " " << idx;
  }

  // Print score of this hypothesis and the number of inliers.
  os << ", score: " << hypothesis.score << ", inliers: " << hypothesis.inliers.size();
  return os;
}

// Print any std::array for debugging.
template <typename T, size_t num_elements>
std::ostream& operator<<(std::ostream& os, const std::array<T, num_elements>& v) {
  for (const T& x : v) {
    os << " " << x;
  }
  return os;
}

// Print any std::vector for debugging.
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
  for (const T& x : v) {
    os << " " << x;
  }
  return os;
}

// Test whether indices within a RANSAC sample set generated by DefaultRansacSampler repeat.
// Elements in a sample set should never repeat.
TEST(DefaultRansacSamplerTest, UniquenessPerSeed) {
  // Sampling parameters.
  constexpr unsigned sample_size = 6;     // Number of items in the sampled subset
  constexpr unsigned input_size = 10;     // Number of input items to sample from
  constexpr unsigned num_samples = 1000;  // Number of independent sample sets to draw
  constexpr unsigned rand_seed = 1256;    // Random seed to initialize the random engine

  // Create a sampler.
  isaac::pnp::DefaultRansacSampler<6> sampler(input_size, rand_seed);

  // Draw many independent subsets (= samples).
  for (unsigned k = 0; k < num_samples; k++) {
    // Draw a subset of unique indices.
    std::array<unsigned, sample_size> subset = sampler();

    // Check each element of the subset.
    for (unsigned i = 0; i < sample_size; i++) {
      // Check the range of the index.
      ASSERT_GE(subset[i], 0);
      ASSERT_LT(subset[i], input_size);

      // Make sure indices do not repeat within the subset.
      for (unsigned j = i + 1; j < sample_size; j++) {
        ASSERT_NE(subset[i], subset[j]);
      }
    }
  }
}

// Test whether each input item has equal chance of being selected into a RANSAC sample set.
// This is a probabilistic test, the chance of failing is non-zero but neglectable.
TEST(DefaultRansacSamplerTest, IsEqualChance) {
  // Create a RANSAC sampler object with random sample.
  constexpr unsigned sample_size = 6;
  constexpr unsigned input_size = 17;
  std::random_device r{};
  isaac::pnp::DefaultRansacSampler<6> sampler(input_size, r());

  // Draw a large number of RANSAC samples (subsets) and count
  // for every possible input index how many times it is chosen.
  // We expect that the result is a uniform distribution meaning that each input sample
  // has equal chance to be selected into the sample set.
  std::vector<double> histogram(input_size, 0);
  unsigned num_samples = 100000;
  for (unsigned k = 0; k < num_samples; k++) {
    // Draw a subset of unique indices.
    std::array<unsigned, sample_size> subset = sampler();

    for (unsigned i = 0; i < sample_size; i++) {
      // Check the range of the index.
      ASSERT_GE(subset[i], 0);
      ASSERT_LT(subset[i], input_size);

      // Count the number of occurences of every possible index.
      histogram[subset[i]]++;
    }
  }

  // Calculate the expected number occurences of each index (uniform distribution expected).
  // There are num_samples * sample_size indices drawn in total out of input_size indices.
  // Each of the input size indices should be selected 'expectation' number of times on average.
  double expectation = double(num_samples * sample_size) / input_size;

  // Measure the largest deviation from the expected number.
  double max_deviation = 0;
  for (unsigned i = 0; i < input_size; i++) {
    double d = std::abs(expectation - histogram[i]);
    if (d > max_deviation) {
      max_deviation = d;
    }
  }

  // Use a hard threshold on the largest deviation.
  // The threshold should be set high enough to reduce the chance of test failure (no guarantees)
  // but low enough to keep the uniformity test meaningful.
  // Print the statistics to the test output.
  std::stringstream ss;
  const double threshold = 0.05;
  ss << "Max deviation from uniform chance: ";
  ss << 100.0 * max_deviation / expectation << "%";
  ss << " (threshold: " << 100.0 * threshold << "%";
  ss << ", samples: " << num_samples << ")";
  LOG_INFO("%s", ss.str().c_str());

  ASSERT_LT(max_deviation / expectation, threshold);
}

// Test pose comparisons implemented in EpnpRansacAdaptor::isSimilar().
TEST(EpnpRansacAdaptorTest, PoseComparisonTests) {
  // Create an EpnpRansacAdaptor object.
  // The parameters passed to the constructor are not important, they are not used by isSimilar().
  isaac::Matrix2d points2;
  isaac::Matrix3d points3;
  isaac::pnp::epnp::EpnpRansacAdaptor adaptor(points3, points2, 1, 1, 0, 0, 0.1);

  // Identical poses.
  double angle = isaac::DegToRad(235.75);
  isaac::Vector3d axis(8.3, -0.3, 1.1);
  isaac::Vector3d axis2(-1.3, -3.3, -0.8);
  isaac::Vector3d position(25.3, -82.1, 39.1234);
  isaac::Pose3d pose1{isaac::SO3d::FromAngleAxis(angle, axis), isaac::Vector3d(8.3, -5.2, 125.6)};
  isaac::Pose3d pose2 = pose1;
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2));

  // Test simultaneous flipping of angle and axis signs, the underlying rotation being identical.
  pose2.rotation = isaac::SO3d::FromAngleAxis(-angle, -axis);
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2));

  // Test handling of two-fold angle-axis ambiguity within [0,2*pi] range.
  pose2.rotation = isaac::SO3d::FromAngleAxis(2 * isaac::Pi<double> - angle, -axis);
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2));

  // Test handling of angle-axis ambiguity modulo 2*pi and flipping.
  pose2.rotation = isaac::SO3d::FromAngleAxis(8 * isaac::TwoPi<double> - angle, -axis);
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2));

  // Test handling of angle-axis ambiguity at Pi.
  pose1.rotation = isaac::SO3d::FromAngleAxis(isaac::Pi<double>, axis);
  pose2.rotation = isaac::SO3d::FromAngleAxis(-isaac::Pi<double>, axis);
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2));

  // Test zero angle with different rotation axes.
  pose1.rotation = isaac::SO3d::FromAngleAxis(0.0, axis);
  pose2.rotation = isaac::SO3d::FromAngleAxis(0.0, axis2);
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2));

  // Test angle Pi with different rotation axes.
  pose1.rotation = isaac::SO3d::FromAngleAxis(isaac::Pi<double>, axis);
  pose2.rotation = isaac::SO3d::FromAngleAxis(isaac::Pi<double>, axis2);
  EXPECT_FALSE(adaptor.isSimilar(pose1, pose2));

  // Test difference in translation only.
  pose1.rotation = isaac::SO3d::FromAngleAxis(angle, axis);
  pose2.rotation = pose1.rotation;
  pose1.translation = -pose1.rotation.matrix() * position;
  pose2.translation = -pose2.rotation.matrix() * (position + isaac::Vector3d(13.25, 0, 0));
  EXPECT_FALSE(adaptor.isSimilar(pose1, pose2, 13.24));
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2, 13.26));

  // Test difference in rotation angle only with a tight threshold.
  // Make sure the camera position is the same (which differs from the translation component).
  pose1.rotation = isaac::SO3d::FromAngleAxis(angle, axis);
  pose2.rotation = isaac::SO3d::FromAngleAxis(angle + isaac::DegToRad(0.1234), axis);
  pose1.translation = -pose1.rotation.matrix() * position;
  pose2.translation = -pose2.rotation.matrix() * position;
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2, 1e-6, 0.1235));
  EXPECT_FALSE(adaptor.isSimilar(pose1, pose2, 1e-6, 0.1233));

  // Test larger difference in rotation angle.
  pose1.rotation = isaac::SO3d::FromAngleAxis(isaac::DegToRad(130.0), axis);
  pose2.rotation = isaac::SO3d::FromAngleAxis(isaac::DegToRad(230.0), axis);
  pose1.translation = -pose1.rotation.matrix() * position;
  pose2.translation = -pose2.rotation.matrix() * position;
  EXPECT_FALSE(adaptor.isSimilar(pose1, pose2, 1e-6, 90.0));
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2, 1e-6, 110.0));

  // Test turnaround for difference in rotation angle only.
  pose1.rotation = isaac::SO3d::FromAngleAxis(isaac::DegToRad(355.0), axis);
  pose2.rotation = isaac::SO3d::FromAngleAxis(isaac::DegToRad(5.0), axis);
  pose1.translation = -pose1.rotation.matrix() * position;
  pose2.translation = -pose2.rotation.matrix() * position;
  EXPECT_FALSE(adaptor.isSimilar(pose1, pose2, 1e-6, 9.9));
  EXPECT_TRUE(adaptor.isSimilar(pose1, pose2, 1e-6, 10.1));

  // test difference in rotation axis only
  pose1.rotation = isaac::SO3d::FromAngleAxis(angle, axis);
  pose2.rotation = isaac::SO3d::FromAngleAxis(angle, axis + isaac::Vector3d(0, 0.1, 1));
  pose1.translation = -pose1.rotation.matrix() * position;
  pose2.translation = -pose2.rotation.matrix() * position;
  EXPECT_FALSE(adaptor.isSimilar(pose1, pose2));
}

// A deterministic dummy RansacSampler to replace DefaultRansacSampler for controlled testing.
// This dummy sampler simply replays a preset sequence of index subsets, each of size 6.
class RansacTestSampler {
 private:
  // Number of input items to sample from. Sampled indices are in the range [0, input_size_].
  unsigned input_size_;

  // Sequence of samples.
  std::vector<std::array<unsigned, 6>> sequence_;

  // Index of the current sample in the sequence during "replay".
  unsigned index_;

 public:
  RansacTestSampler(unsigned input_size) : input_size_(input_size), index_(0) {}

  // Accessors
  unsigned sampleSize() const { return 6; }
  unsigned inputSize() const { return input_size_; }
  unsigned sequenceSize() const { return sequence_.size(); }

  // Add a sample to the end of the sequence.
  void Add(std::array<unsigned, 6>&& sample) { sequence_.push_back(std::move(sample)); }

  // Access elements of the sequence.
  const std::array<unsigned, 6>& operator[](unsigned i) const { return sequence_[i]; }

  // Return the next sample of the sequence.
  std::array<unsigned, 6> operator()() {
    auto& sample = sequence_[index_ % sequence_.size()];
    index_ = (index_ + 1) % sequence_.size();
    return sample;
  }
};

// Print elements of any container in C++ compatible format.
// Used in PrintRansacTestData() to generate data for a test fixture.
template <typename Iter>
void Dump(Iter begin, Iter end) {
  Iter it = begin;
  while (it != end) {
    std::cout << *it;
    ++it;
    if (it != end) {
      std::cout << ",";
    }
  }
}

// Print elements of a matrix columnwise in a single line in C++ compatible format.
// Used in PrintRansacTestData() to generate data for a test fixture.
void DumpMatrix(const isaac::MatrixXd& mat) {
  for (int i = 0; i < mat.rows(); i++) {
    std::cout << "  ";
    for (int j = 0; j < mat.cols(); j++) {
      std::cout << mat(i, j);

      // Elements are separated by a comma and the last element is followed by a semicolon.
      if (i == mat.rows() - 1 && j == mat.cols() - 1) {
        std::cout << ";";
      } else {
        std::cout << ", ";
      }
    }
    std::cout << std::endl;
  }
}

// Use this function to generate and print random test data for RANSAC tests.
// The output can be copy-pasted into EpnpRansacTest::SetUp() to use in the test fixture.
// Generates points from two incompatible poses plus some extra global outliers for testing RANSAC.
// Points 0,1...6  are inliers to model1 (pose1).
// Points 7,8...12 are inliers to model2 (pose2).
// Points 13, 14 are outliers to both models.
void PrintRansacTestData() {
  // Generate two cameras with random pose but fixed and identical intrinsics.
  const int width = 1280;
  const int height = 720;
  const double focal = 700.0;
  isaac::pnp::Camera camera1 = isaac::pnp::GenerateRandomCamera(width, height, focal);
  isaac::pnp::Camera camera2 = isaac::pnp::GenerateRandomCamera(width, height, focal);
  isaac::Vector3d axis1 = isaac::pnp::AngleAxisFromMatrix(camera1.rotation_matrix);
  isaac::Vector3d axis2 = isaac::pnp::AngleAxisFromMatrix(camera2.rotation_matrix);

  // Print all parameters of the two cameras in C++ code format.
  std::cout.unsetf(std::ios::fixed);
  std::cout << "<---------------------------------- start test data" << std::endl;
  std::cout << "// width  = " << width << std::endl;
  std::cout << "// height = " << height << std::endl;
  std::cout << "focal_u = focal_v = " << focal << ";" << std::endl;
  std::cout << "principal_u = " << camera1.calib_matrix(0, 2) << ";" << std::endl;
  std::cout << "principal_v = " << camera1.calib_matrix(1, 2) << ";" << std::endl;
  std::cout << std::ios::fixed << std::setprecision(12);
  std::cout << "gt_axis1 = isaac::Vector3d(";
  std::cout << axis1(0) << "," << axis1(1) << "," << axis1(2) << ");" << std::endl;
  std::cout << "gt_axis2 = isaac::Vector3d(";
  std::cout << axis2(0) << "," << axis2(1) << "," << axis2(2) << ");" << std::endl;
  std::cout << "gt_position1  = isaac::Vector3d(";
  std::cout << camera1.position(0) << ",";
  std::cout << camera1.position(1) << ",";
  std::cout << camera1.position(2) << ");" << std::endl;
  std::cout << "gt_position2  = isaac::Vector3d(";
  std::cout << camera2.position(0) << ",";
  std::cout << camera2.position(1) << ",";
  std::cout << camera2.position(2) << ");" << std::endl;

  // Generate equal number of inlier 2D-3D matches for each of the two camera models.
  // For each camera, its 3D points lie within the camera FoV between the near and far planes.
  // Stack the two generated 2D/3D points sets into a single 2xN/3xN matrix.
  const unsigned num_inliers = 7;
  const double near = 2.0;
  const double far = 6.0;
  isaac::Matrix3Xd points3_model1, points3_model2;
  isaac::Matrix2Xd points2_model1, points2_model2;
  isaac::pnp::GenerateFovPoints(num_inliers, camera1, near, far, &points3_model1, &points2_model1);
  isaac::pnp::GenerateFovPoints(num_inliers, camera2, near, far, &points3_model2, &points2_model2);
  isaac::Matrix3Xd points3 = isaac::pnp::StackMatricesHorizontally(points3_model1, points3_model2);
  isaac::Matrix2Xd points2 = isaac::pnp::StackMatricesHorizontally(points2_model1, points2_model2);

  // Insert 2D-3D matches that are outliers to both models.
  const unsigned num_outliers = 2;
  const double radius = 30.0;
  InsertOutliers(num_outliers, camera1, radius, &points3, &points2, false);

  // Collect the indices of inliers for each of the two models separately.
  // A 2D/3D match is inlier if its reprojection error in pixels is neglectable.
  const double threshold = 0.01;
  isaac::VectorXd errs1 =
      isaac::pnp::ColwiseNorms(points2 - ProjectPoints(camera1, points3, true, true));
  isaac::VectorXd errs2 =
      isaac::pnp::ColwiseNorms(points2 - ProjectPoints(camera2, points3, true, true));
  std::vector<unsigned> inliers1;
  std::vector<unsigned> inliers2;
  for (unsigned i = 0; i < errs1.size(); i++) {
    if (errs1[i] < threshold) {
      inliers1.push_back(i);
    }
  }
  for (unsigned i = 0; i < errs2.size(); i++) {
    if (errs2[i] < threshold) {
      inliers2.push_back(i);
    }
  }

  // Print the indices of the inliers in C++ code format.
  std::cout << "inliers1 = vector<unsigned>{";
  Dump(inliers1.begin(), inliers1.end());
  std::cout << "};" << std::endl;
  std::cout << "inliers2 = vector<unsigned>{";
  Dump(inliers2.begin(), inliers2.end());
  std::cout << "};" << std::endl;

  // Print the 2D and 3D points in C++ code format.
  // The point matrices are printed transposed (Nx2 and Nx3)
  // to make the output coordinates more readable.
  std::cout << std::ios::fixed << std::setprecision(12);
  std::cout << "isaac::MatrixXd pts3(" << points3.cols() << ", " << points3.rows() << ");";
  std::cout << std::endl;
  std::cout << "pts3 << " << std::endl;
  DumpMatrix(points3.transpose());
  std::cout << "points3 = pts3.transpose();" << std::endl;
  std::cout << "isaac::MatrixXd pts2(" << points2.cols() << ", " << points2.rows() << ");";
  std::cout << std::endl;
  std::cout << "pts2 << " << std::endl;
  DumpMatrix(points2.transpose());
  std::cout << "points2 = pts2.transpose();" << std::endl;
  std::cout << "<---------------------------------- end test data" << std::endl;
}

// Test fixture for testing RANSAC-based pose estimation.
// Stores ground-truth data for two synthetic cameras with identical intrinsics but different poses
// and a set of 2D/3D point matches with 3 subsets:
// (1) inlier matches to the first camera model,
// (2) inlier matches to the second camera model,
// (3) outliers to both models.
class EpnpRansacTest : public ::testing::Test {
 protected:
  // Identical intrinsic camera parameters for the two cameras.
  double focal_u, focal_v, principal_u, principal_v;

  // Joint set of 2D/3D point matches with inliers and outliers mixed together.
  isaac::Matrix2Xd points2;
  isaac::Matrix3Xd points3;
  std::vector<unsigned> inliers1;
  std::vector<unsigned> inliers2;

  // Ground truth poses of the two camera models.
  isaac::Vector3d gt_axis1;
  isaac::Vector3d gt_axis2;
  isaac::Vector3d gt_position1;
  isaac::Vector3d gt_position2;

 public:
  void SetUp() override {
    // Generate and dump new random test data. Copy the output and paste it below.
    // PrintRansacTestData();

    // The rest of this function is code generated by PrintRansacTestData().
    focal_u = focal_v = 700.0;
    principal_u = 640.0;
    principal_v = 360.0;
    gt_axis1 = isaac::Vector3d(1.246431082136, -1.488785282762, -1.717268329543);
    gt_axis2 = isaac::Vector3d(-1.474963839733, -0.393161160840, -0.552226884507);
    gt_position1 = isaac::Vector3d(40.959062410363, -49.812992848765, 48.298686644820);
    gt_position2 = isaac::Vector3d(90.028174426100, -18.458540753177, -80.312594442311);
    inliers1 = std::vector<unsigned>{0, 1, 2, 3, 4, 5, 6};
    inliers2 = std::vector<unsigned>{7, 8, 9, 10, 11, 12, 13};

    isaac::MatrixXd pts3(16, 3);
    pts3 << 41.400241668095, -44.804710827040, 51.713684730464, 39.430519169917, -44.348882210449,
        51.737095051124, 38.976047959394, -47.853245718666, 48.035537371287, 40.121192878014,
        -46.566809657102, 48.723233147234, 42.146531276923, -43.742220747133, 49.151236907191,
        39.689011504827, -45.123227520864, 52.569910421456, 40.532981841159, -44.985869661988,
        45.874549415954, 91.634076934706, -22.765244053848, -80.184494647314, 95.364222887767,
        -20.116176008183, -78.723192969495, 90.732603393096, -20.884428333034, -80.228258881278,
        95.769983747879, -21.320357375152, -81.698183257387, 94.628990286377, -21.090631619923,
        -80.527450089501, 89.974729650745, -25.308855163863, -77.586708526601, 94.514488443643,
        -21.030260810473, -78.703926460615, 49.361946471414, -59.553509542194, 32.889233874131,
        32.523546008870, -61.005788561317, 53.299292341019;
    points3 = pts3.transpose();

    isaac::MatrixXd pts2(16, 2);
    pts2 << 12.395121723710, 354.059751504487, 220.188841927345, 556.325084028859, 853.730568087874,
        675.275631428689, 522.223310954897, 386.704006604826, 347.281595704401, 89.307240469900,
        36.122163391302, 643.209326222118, 862.781555544812, 84.769172394853, 461.035680577970,
        346.450723118805, 1191.366827535621, 513.753105427619, 405.180431127773, 355.671223505314,
        1013.206030748049, 102.744712993492, 968.988030487053, 251.735658969613, 188.308022960250,
        681.152710697888, 981.848664116682, 520.116868250137, 354.655750007339, 315.040405692242,
        159.349736752775, 486.784479513776;
    points2 = pts2.transpose();
  }
};

// Verify if a pose hypothesis returned by RANSAC is correct, assuming no noise in
// the 2D/3D matches leading to this pose hypothesis.
//  (1) Test if the sample is reported correctly inside the hypothesis.
//  (2) Test if all actual inliers are found and reported within the hypothesis.
//  (3) Test if the hypothesized pose matches the ground-truth pose.
// Parameters:
//  hypothesis     The RANSAC pose hypothesis to test.
//  seed           The ground-truth sample fed to RANSAC in the test, leading to this hypothesis.
//                 This sample is randomly generated inside RANSAC during normal operation.
//  inliers        The known set of inliers RANSAC is supposed to find a part of this hypothesis.
//  gt_angle_axis  Ground-truth camera orientation as an angle-axis.
//  gt_position    Ground-truth camera position.
void CheckHypothesis(const isaac::pnp::RansacHypothesis<isaac::Pose3d, 6>& hypothesis,
                     const std::array<unsigned, 6>& seed, const std::vector<unsigned int>& inliers,
                     const isaac::Vector3d& gt_angle_axis, const isaac::Vector3d& gt_position) {
  // Test if the sample that generated the hypothesis and the inliers compatible with the hypothesis
  // are the expected ones.
  ASSERT_EQ(hypothesis.seed.size(), 6);
  ASSERT_EQ(hypothesis.inliers.size(), inliers.size());
  ASSERT_NEAR(hypothesis.score, double(inliers.size()), 1e-3);
  ASSERT_TRUE((hypothesis.inliers == inliers));
  ASSERT_TRUE((hypothesis.seed == seed));

  // Wrap the norm of the angle-axis in the [0,Pi] range without changing the underlying rotation.
  double angle = std::fmod(hypothesis.model.rotation.angle(), 2 * isaac::Pi<double>);
  if (angle < 0.0) angle += 2 * isaac::Pi<double>;
  isaac::Vector3d angle_axis = hypothesis.model.rotation.axis();
  if (angle > isaac::Pi<double>) {
    angle = 2 * isaac::Pi<double> - angle;
    angle_axis = -angle_axis;
  }
  angle_axis *= angle;

  // Extract the camera position from the pose.
  isaac::Vector3d position =
      -isaac::pnp::MatrixFromAngleAxis(angle_axis).transpose() * hypothesis.model.translation;

  // Test if the hypothesized pose matches the ground-truth pose to numerical precision.
  // This assumes no noise in the measurement.
  const double tol = 1e-6;
  ASSERT_LT((position - gt_position).norm(), tol);
  // Handle the remaining flip ambiguity of angle-axes at rotation angle Pi.
  // Angle axes with norm Pi are equal if they are parallel.
  if (std::abs(angle - isaac::Pi<double>) < tol &&
      std::abs(gt_angle_axis.norm() - isaac::Pi<double>) < tol) {
    // Parallelity test.
    ASSERT_LT(angle_axis.cross(gt_angle_axis).norm(), tol);
  } else {
    // Equality test for the angle range [0,Pi), excluding Pi.
    // The angle-axis representation is unique in this range.
    ASSERT_LT((angle_axis - gt_angle_axis).norm(), tol);
  }
}

// Test RANSAC with the noise-free test data in EpnpRansacTest.
// Sampling sequence: one all-inlier sample.
// Expected result: there should be a single correct hypothesis returned for the right model.
TEST_F(EpnpRansacTest, SingleGoodHypothesis) {
  // RANSAC parameters.
  const unsigned max_top_hypotheses = 5;
  const double threshold = 1e-6;  // Reprojection error in pixels for an inlier.

  // Set up RANSAC sampling sequence that replaces random sampling in the tests.
  RansacTestSampler sampler(points3.cols());
  sampler.Add({0, 1, 2, 3, 4, 5});  // good hypothesis for model1

  // Run RANSAC with test fixture data.
  isaac::pnp::epnp::EpnpRansacAdaptor adaptor(points3, points2, focal_u, focal_v, principal_u,
                                              principal_v, threshold);
  auto top_hypotheses =
      isaac::pnp::Ransac(adaptor, sampler.sequenceSize(), max_top_hypotheses, &sampler);

  // Test if there is a singe accepted hypothesis and that it is perfect.
  ASSERT_EQ(top_hypotheses.size(), 1);
  SCOPED_TRACE("SingleGoodHypothesis");
  CheckHypothesis(top_hypotheses[0], sampler[0], inliers1, gt_axis1, gt_position1);
}

// Test RANSAC with the noise-free test data in EpnpRansacTest.
// Sampling sequence: only one contaminated sample.
// Expected result: RANSAC is not supposed to find any solution.
TEST_F(EpnpRansacTest, SingleBadHypothesis) {
  // RANSAC parameters.
  const unsigned max_top_hypotheses = 5;
  const double threshold = 0.1;

  // Set up RANSAC sampling sequence that replaces random sampling in the tests.
  RansacTestSampler sampler(points3.cols());
  sampler.Add({0, 1, 2, 3, 9, 14});  // bad hypothesis for both models

  // Run RANSAC with test fixture data.
  isaac::pnp::epnp::EpnpRansacAdaptor adaptor(points3, points2, focal_u, focal_v, principal_u,
                                              principal_v, threshold);
  auto top_hypotheses =
      isaac::pnp::Ransac(adaptor, sampler.sequenceSize(), max_top_hypotheses, &sampler);

  // Solution should not be found from a single bad hypothesis.
  ASSERT_EQ(top_hypotheses.size(), 0);
}

// Test RANSAC with the noise-free test data in EpnpRansacTest.
// Sampling sequence: many all-inlier samples.
// Expected result: RANSAC should recognize internally that the samples lead to the same result
// and return it only once. This result should be the perfect solution.
TEST_F(EpnpRansacTest, RepeatingGoodHypotheses) {
  // RANSAC parameters.
  const unsigned max_top_hypotheses = 5;
  const double threshold = 0.1;

  // Set up RANSAC sampling sequence that replaces random sampling in the tests.
  RansacTestSampler sampler(points3.cols());
  sampler.Add({1, 2, 3, 4, 5, 6});  // good hypothesis for model1
  sampler.Add({0, 2, 3, 4, 5, 6});  // good hypothesis for model1
  sampler.Add({0, 1, 3, 4, 5, 6});  // good hypothesis for model1
  sampler.Add({0, 1, 2, 4, 5, 6});  // good hypothesis for model1
  sampler.Add({0, 1, 2, 3, 5, 6});  // good hypothesis for model1
  sampler.Add({0, 1, 2, 3, 4, 6});  // good hypothesis for model1
  sampler.Add({0, 1, 2, 3, 4, 5});  // good hypothesis for model1

  // Run RANSAC with test fixture data.
  isaac::pnp::epnp::EpnpRansacAdaptor adaptor(points3, points2, focal_u, focal_v, principal_u,
                                              principal_v, threshold);
  auto top_hypotheses =
      isaac::pnp::Ransac(adaptor, sampler.sequenceSize(), max_top_hypotheses, &sampler);

  // There should be a single correct hypothesis returned because the different sample sets
  // will lead to the same correct inlier set, hence, the same pose. Check this hypothesis.
  ASSERT_EQ(top_hypotheses.size(), 1);
  SCOPED_TRACE("RepeatingGoodHypotheses");
  CheckHypothesis(top_hypotheses[0], sampler[0], inliers1, gt_axis1, gt_position1);
}

// Test RANSAC with the noise-free test data in EpnpRansacTest.
// Sampling sequence: one perfect sample per model.
// Expected result: RANSAC is supposed to find both correct models.
TEST_F(EpnpRansacTest, OneGoodHypothesisPerModel) {
  // RANSAC parameters.
  const unsigned max_top_hypotheses = 5;
  const double threshold = 0.1;

  // Set up RANSAC sampling sequence that replaces random sampling in the tests.
  RansacTestSampler sampler(points3.cols());
  sampler.Add({0, 1, 2, 3, 4, 5});     // good hypothesis for model1
  sampler.Add({7, 8, 9, 10, 11, 12});  // good hypothesis for model2

  // Run RANSAC with test fixture data.
  isaac::pnp::epnp::EpnpRansacAdaptor adaptor(points3, points2, focal_u, focal_v, principal_u,
                                              principal_v, threshold);
  auto top_hypotheses =
      isaac::pnp::Ransac(adaptor, sampler.sequenceSize(), max_top_hypotheses, &sampler);

  // Check if two solutions are found: one accepted hypothesis for each of the two valid poses.
  ASSERT_EQ(top_hypotheses.size(), 2);
  SCOPED_TRACE("OneGoodHypothesisPerModel");
  CheckHypothesis(top_hypotheses[0], sampler[0], inliers1, gt_axis1, gt_position1);
  CheckHypothesis(top_hypotheses[1], sampler[1], inliers2, gt_axis2, gt_position2);
}

// Test RANSAC with the noise-free test data in EpnpRansacTest.
// Sampling sequence: only contaminated samples.
// Expected result: RANSAC is not supposed to find the correct solution.
TEST_F(EpnpRansacTest, OnlyBadHypotheses) {
  // RANSAC parameters.
  const unsigned max_top_hypotheses = 5;
  const double threshold = 0.1;

  // Set up RANSAC sampling sequence that replaces random sampling in the tests.
  RansacTestSampler sampler(points3.cols());
  sampler.Add({0, 1, 2, 3, 4, 10});    // bad hypothesis for both models
  sampler.Add({0, 1, 2, 3, 4, 10});    // bad hypothesis for both models
  sampler.Add({3, 8, 9, 10, 11, 12});  // bad hypothesis for both models
  sampler.Add({3, 5, 9, 11, 13, 15});  // bad hypothesis for both models

  // Run RANSAC with test fixture data.
  isaac::pnp::epnp::EpnpRansacAdaptor adaptor(points3, points2, focal_u, focal_v, principal_u,
                                              principal_v, threshold);
  auto top_hypotheses =
      isaac::pnp::Ransac(adaptor, sampler.sequenceSize(), max_top_hypotheses, &sampler);

  // No hypothesis should be accepted since all samples were contaminated by outliers.
  ASSERT_EQ(top_hypotheses.size(), 0);
}

// Test RANSAC with the noise-free test data in EpnpRansacTest.
// Sampling sequence: a mixture of contaminated samples and several clean samples for both models.
// Expected result: RANSAC should return a single accepted hypothesis per model.
TEST_F(EpnpRansacTest, ManyMixedHypotheses) {
  // RANSAC parameters.
  const unsigned max_top_hypotheses = 5;
  const double threshold = 0.1;

  // Set up RANSAC sampling sequence that replaces random sampling in the tests.
  RansacTestSampler sampler(points3.cols());
  sampler.Add({0, 1, 2, 3, 4, 10});     // bad hypothesis for both models
  sampler.Add({0, 1, 2, 3, 4, 10});     // bad hypothesis for both models
  sampler.Add({7, 8, 9, 10, 11, 12});   // good hypothesis for model2
  sampler.Add({3, 8, 9, 10, 11, 12});   // bad hypothesis for both models
  sampler.Add({3, 5, 9, 11, 13, 14});   // bad hypothesis for both models
  sampler.Add({0, 1, 2, 4, 5, 6});      // good hypothesis for model1
  sampler.Add({3, 5, 9, 11, 13, 14});   // bad hypothesis for both models
  sampler.Add({2, 4, 8, 10, 11, 13});   // bad hypothesis for both models
  sampler.Add({8, 9, 10, 11, 12, 13});  // good hypothesis for model2
  sampler.Add({1, 2, 3, 4, 5, 6});      // good hypothesis for model1

  // Run RANSAC with test fixture data.
  isaac::pnp::epnp::EpnpRansacAdaptor adaptor(points3, points2, focal_u, focal_v, principal_u,
                                              principal_v, threshold);
  auto top_hypotheses =
      isaac::pnp::Ransac(adaptor, sampler.sequenceSize(), max_top_hypotheses, &sampler);

  // Both models were sampled in the sequence several times, so both models should be returned
  // as one hypothesis each. No hypothesis should repeat.
  ASSERT_EQ(top_hypotheses.size(), 2);
  SCOPED_TRACE("ManyMixedHypotheses");
  CheckHypothesis(top_hypotheses[0], sampler[2], inliers2, gt_axis2, gt_position2);
  CheckHypothesis(top_hypotheses[1], sampler[5], inliers1, gt_axis1, gt_position1);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
