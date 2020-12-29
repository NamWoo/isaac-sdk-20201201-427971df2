/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/gems/math/catmull_rom.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(CatmullRomSpline, One) {
  using Vector1d = Vector<double, 1>;
  const Vector1d values[] = {Vector1d{1.0}, Vector1d{2.0}, Vector1d{1.0}, Vector1d{2.0}};
  double times[] = {1.0, 2.0, 3.0, 4.0};
  for (double t = 0.0; t <= 5.0; t += 0.05) {
    Vector1d v, d;
    CatmullRomSplineEvaluate(values, times, t, &v, &d);
    const double expected_v = -26.0 + 36.0*t - 15.0*t*t + 2.0*t*t*t;
    ASSERT_NEAR(v[0], expected_v, 1e-9);
    const double expected_d = 36.0 - 30.0*t + 6.0*t*t;
    ASSERT_NEAR(d[0], expected_d, 1e-9);
  }
}

TEST(CatmullRomSpline, ZeroTangent) {
  const CatmullRomSpline<double, 2> spline({Vector2d{1.0, 1.0}, Vector2d{2.0, 2.0},
                                            Vector2d{1.0, 1.0}, Vector2d{2.0, 2.0}});
  EXPECT_NEAR(spline.sampleFrame(0.0).col(1).norm(), 1.0, 1e-9);
}

TEST(CatmullRomSpline, CurveTimes) {
  // Take delta steps of 0.25, 1.0 and 2.25 along the vector {2,1}. This will result in delta curve
  // time steps of 0.5, 1.0 and 1.5.
  const Vector2d points[4] = {
      Vector2d{0, 0}, Vector2d{0.5, 0.25}, Vector2d{2.5, 1.25}, Vector2d{7.0, 3.5}};
  double times[4];
  times[0] = 1.3;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, 4));
  const double expected_step = std::pow(5.0, 0.25);
  EXPECT_NEAR(times[0],                      1.3, 1e-9);
  EXPECT_NEAR(times[1], 0.50*expected_step + 1.3, 1e-9);
  EXPECT_NEAR(times[2], 1.50*expected_step + 1.3, 1e-9);
  EXPECT_NEAR(times[3], 3.00*expected_step + 1.3, 1e-9);
}

TEST(CatmullRomSpline, CurveTimesRepeating) {
  // Check that computing curve times for list with duplicated points is invalid.
  const Vector2d points[4] = {
      Vector2d{0, 0}, Vector2d{0.5, 0.25}, Vector2d{0.5, 0.25}, Vector2d{3.0, 4.5}};
  double times[4];
  times[0] = 1.3;
  ASSERT_FALSE(CatmullRomComputeCurveTimes(points, times, 4));
}

TEST(CatmullRomSpline, InterpolateLinearSpline) {
  // Check interpolation for a short linear curve.
  const Vector2d points[4] = {
      Vector2d{0, 0}, Vector2d{2, 1}, Vector2d{4, 2}, Vector2d{6, 3}};
  double times[4];
  times[0] = 0.0;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, 4));
  for (double t = 0.0; t <= 1.0; t += 0.1) {
    Vector2d position;
    CatmullRomSplineEvaluate(points, times, (1.0 - t)*times[1] + t*times[2], &position);
    ISAAC_ASSERT_VEC_NEAR((1.0 - t)*points[1] + t*points[2], position, 1e-9);
  }
}

TEST(CatmullRomSpline, EvaluateAtPoints4) {
  // Check interpolation at sample points using 4-point version. The spline is guaranteed to go
  // through the second and third sample point.
  const Vector2d points[4] = {
      Vector2d{0.303656, 0.316798}, Vector2d{0.677207, 1.17211},
      Vector2d{0.896317, 1.72843}, Vector2d{1.0771, 1.91572}};
  double times[4];
  times[0] = 0.0;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, 4));
  Vector2d position;
  CatmullRomSplineEvaluate(points, times, times[1], &position);
  ISAAC_ASSERT_VEC_NEAR(points[1], position, 1e-9);
  CatmullRomSplineEvaluate(points, times, times[2], &position);
  ISAAC_ASSERT_VEC_NEAR(points[2], position, 1e-9);
}

TEST(CatmullRomSpline, Diagonal) {
  const Vector2d points[4] = { Vector2d{-1.0, -1.0}, Vector2d{0.0, 0.0},
                               Vector2d{1.0, 1.0}, Vector2d{2.0, 2.0} };
  for (double knot : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    const double delta = std::pow(2.0, 0.5 * knot);  // ||{1, 1}||^knot
    double times[4];
    times[0] = -delta;
    ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, 4, knot));
    for (int i = 0; i < 4; i++) {
      EXPECT_NEAR(times[i], static_cast<double>(i - 1) * delta, 1e-9);
    }
    const Vector2d expected_tangent = Vector2d{1.0 / delta, 1.0 / delta};
    for (double t = 0.0; t <= 1.0; t += 0.1) {
      Vector2d position, tangent;
      CatmullRomSplineEvaluate(points, times, t*delta, &position, &tangent);
      ISAAC_ASSERT_VEC_NEAR(Vector2d(t, t), position, 1e-9);
      ISAAC_ASSERT_VEC_NEAR(expected_tangent, tangent, 1e-9);
    }
  }
}

TEST(CatmullRomSpline, EvaluateAtPointsN) {
  constexpr int kNumPoints = 12;
  // Check interpolation at sample points using N-point version. The spline is guaranteed to go
  // through all sample points except for the first and the last point.
  const Vector2d points[kNumPoints] = {
      Vector2d{0.234099, 0.893742}, Vector2d{1.14441, 1.3151}, Vector2d{1.62298, 1.70618},
      Vector2d{2.40708, 2.38953}, Vector2d{2.97525, 2.50614}, Vector2d{3.90171, 2.62645},
      Vector2d{4.3033, 3.38146}, Vector2d{4.73817, 3.80319}, Vector2d{5.42781, 4.4778},
      Vector2d{6.22677, 5.42769}, Vector2d{6.47075, 6.25194}, Vector2d{7.22324, 6.84748}};
  double times[kNumPoints];
  times[0] = 0.0;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, kNumPoints));
  for (size_t i = 1; i + 1 < kNumPoints; i++) {
    const int index = CatmullRomSplineIndex(points, times, kNumPoints, times[i]);
    ASSERT_EQ(index, (i + 2 == kNumPoints) ? i - 2 : i - 1);
    Vector2d position;
    CatmullRomSplineEvaluate(points + index, times + index, times[i], &position);
    ISAAC_ASSERT_VEC_NEAR(points[i], position, 1e-9);
  }
}

TEST(CatmullRomSpline, Class) {
  const std::vector<Vector2d> points{
      Vector2d{0.234099, 0.893742}, Vector2d{1.14441, 1.3151}, Vector2d{1.62298, 1.70618},
      Vector2d{2.40708, 2.38953}, Vector2d{2.97525, 2.50614}, Vector2d{3.90171, 2.62645},
      Vector2d{4.3033, 3.38146}, Vector2d{4.73817, 3.80319}, Vector2d{5.42781, 4.4778},
      Vector2d{6.22677, 5.42769}, Vector2d{6.47075, 6.25194}, Vector2d{7.22324, 6.84748}};
  const CatmullRomSpline<double, 2> spline(points);
  ASSERT_TRUE(spline.valid());
  for (size_t i = 1; i < spline.size(); i++) {
    ASSERT_LT(spline.keypoint(i - 1).first, spline.keypoint(i).first);
  }
  for (size_t i = 1; i + 1 < spline.size(); i++) {
    Vector2d point;
    double time;
    std::tie(time, point) = spline.keypoint(i);
    ISAAC_ASSERT_VEC_NEAR(point, points[i], 1e-9);
    ISAAC_ASSERT_VEC_NEAR(point, spline.sample(time), 1e-9);
  }
}

}  // namespace isaac
