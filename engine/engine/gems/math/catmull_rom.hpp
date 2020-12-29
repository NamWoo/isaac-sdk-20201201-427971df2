/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {

// Evaluates a Catmull-Rom spline at given curve time `time`. This version is for a minimal curve
// with four control points. Both `points` and `times` must point two four elements. Note that
// the strict ordering t0 < t1 < t2 < t3 must hold true, otherwise division by 0 will occur.
// `time` must not necessarily be in the range `[t0 | t3]`. If `position` is not null the spline
// position at `time` will be computed and stored. If `forward` is not null the spline tangent
// vector at `time` will be computed and stored. `forward` is not necessarily normalized.
//
// The complexity of this algorithm is constant.
template <typename K, int N>
void CatmullRomSplineEvaluate(const Vector<K, N>* points, const K* times, K time,
                              Vector<K, N>* position, Vector<K, N>* tangent = nullptr) {
  const K t0 = times[0];
  const K t1 = times[1];
  const K t2 = times[2];
  const K t3 = times[3];
  const K d0 = time - t0;
  const K d1 = time - t1;
  const K d2 = time - t2;
  const K d3 = time - t3;
  const K d10 = t1 - t0;
  const K d20 = t2 - t0;
  const K d21 = t2 - t1;
  const K d31 = t3 - t1;
  const K d32 = t3 - t2;
  const K d21_d31 = d21*d31;
  const K d20_d21 = d20*d21;
  if (position != nullptr) {
    const K d1_d2 = d1*d2;
    const K d0_d2_d2 = d0*d2*d2;
    const K d1_d1_d3 = d1*d1*d3;
    const K w0 = - d1_d2*d2 / (d10*d20_d21);
    const K w1 = d0_d2_d2 / (d20_d21*d21) + d0_d2_d2 / (d10*d20_d21) + d1_d2*d3 / (d21*d21_d31);
    const K w2 = - d0*d1_d2 / (d20_d21*d21) - d1_d1_d3 / (d21*d21_d31) - d1_d1_d3 / (d21_d31*d32);
    const K w3 = d1*d1_d2 / (d21_d31*d32);
    *position = w0*points[0] + w1*points[1] + w2*points[2] + w3*points[3];
  }
  if (tangent != nullptr) {
    const K k3_t = K{3}*time;
    const K k2_t0 = K{2}*t0;
    const K k2_t1 = K{2}*t1;
    const K k2_t2 = K{2}*t2;
    const K k2_t3 = K{2}*t3;
    const K w0 = - d2*(k3_t - k2_t1 - t2) / (d10*d20_d21);
    const K w1 = d2*(k3_t - k2_t0 - t2) / d20_d21 * (K{1}/d21 + K{1}/d10)
               + (time*(k3_t - k2_t1 - k2_t2 - k2_t3) + t1*(t2 + t3) + t2*t3)/(d21*d21_d31);
    const K w2 = - d1*(k3_t - k2_t3 - t1) / d21_d31 * (K{1}/d21 + K{1}/d32)
               - (time*(k3_t - k2_t0 - k2_t1 - k2_t2) + t0*(t1 + t2) + t1*t2)/(d21*d20_d21);
    const K w3 = d1*(k3_t - k2_t2 - t1) / (d21_d31*d32);
    *tangent = w0*points[0] + w1*points[1] + w2*points[2] + w3*points[3];
  }
}

// Finds the keypoint index which can be used to evaluate a Catmull-Rom spline at the given curve
// time `time`. The returned index can for example be used with `CatmullRomSplineEvaluate` to sample
// the spline. The values in `times` must be in strictly increasing order.
//
// We would like to find `index` such that:
//   times[index] < times[index + 1] <= time < times[index + 2] < times[index + 3]
// `index` is clamped to the range [0 | count - 3[ sucht that there are always four sample points
// available.
//
// The complexity of this algorithm is `log(count)`.
template <typename K, int N>
int CatmullRomSplineIndex(const Vector<K, N>* points, const K* times, int count, K time) {
  ASSERT(count >= 4, "Not enough sample points. Need at least 4, but got %zd", count);
  // Cases: 1) time < times[0]                  => return 0
  //        2) times[n - 1] < time              => return n - 4
  //        3) times[i - 1] < time  < times[i]  => return clamp(i - 2, 0, n - 4)
  //        4) times[i - 1] < time == times[i]  => return clamp(i - 1, 0, n - 4)
  if (time < times[2]) {  // As an additional optimization we don't search for the lower bound
                          // if we would return 0 anyway.
    return 0;  // 1)
  }
  if (time >= times[count - 3]) {  // As an additional optimization we don't search for the lower
                                   // bound if we would return count - 4 anyway.
    return count - 4;  // 2)
  }
  int index = std::lower_bound(times, times + count, time) - times;
  if (time < times[index]) {
    index -= 2;  // 3)
  } else {
    index--;  // 4)
  }
  return index;
}

// Computes curve times based on control points. The `knot` parameter defines the shape of the
// curve. For a centripetal spline choose 0.5 (default), for a uniform spline choose 0, for a
// chordal spline choose 1. Memory for `times` must be allocated be the caller.
//
// The first element of `times` *must be initialized* to the desired start curve time.
//
// If two consecutive points are identical false is returned.
template <typename K, int N>
bool CatmullRomComputeCurveTimes(const Vector<K, N>* points, K* times, int count,
                                 K knot = K(0.5)) {
  ASSERT(K(0) <= knot && knot <= K(1), "`knot` parameter (%f) must be in range [0, 1]", knot);
  // We would like to compute ||d||^knot = (d*d)^(1/2)^knot = (d*d)^(knot/2).
  const K exponent = knot / K(2);
  for (int i = 1; i < count; i++) {
    const K delta = std::pow((points[i] - points[i - 1]).squaredNorm(), exponent);
    if (IsAlmostZero(delta)) return false;
    times[i] = times[i - 1] + delta;
  }
  return true;
}

// A Catmull-Rom spline
template <typename K, int N>
class CatmullRomSpline {
 public:
  // Vector type used for keypoint values
  using vector_t = Vector<K, N>;

  // Creates an empty spline
  CatmullRomSpline() {}

  // Creates a spline based on the given keypoints. If less than 4 keypoints are given this will
  // result in an invalid spline.
  CatmullRomSpline(std::vector<vector_t> keypoints, K knot = K(0.5))
  : keypoint_values_(std::move(keypoints)), knot_(knot) {
    ASSERT(valid(), "Not enough keypoints");
    keypoint_times_.resize(keypoint_values_.size());
    keypoint_times_[0] = K{0};
    CatmullRomComputeCurveTimes(keypoint_values_.data(), keypoint_times_.data(),
                                keypoint_values_.size(), knot_);
    const double offset = keypoint_times_[1];
    for (size_t i = 0; i < keypoint_times_.size(); i++) {
      keypoint_times_[i] -= offset;
    }
  }

  // The total number of keypoints. This includes first and last keypoint.
  size_t size() const { return keypoint_values_.size(); }

  // Returns true if the spline has enough keypoints, i.e. at least 4.
  bool valid() const { return size() >= 4; }

  // The length of the spline in curve time
  K length() const {
    return valid() ? keypoint_times_[keypoint_times_.size() - 2] : K{0};
  }

  // The `knot` value describing the curvature of the spline
  K knot() const { return knot_; }

  const std::vector<vector_t>& getKeypointValues() const { return keypoint_values_; }
  const std::vector<double>& getKeypointTimes() const { return keypoint_times_; }

  // Computes the spline position at the given curve time.
  vector_t sample(K time) const {
    const int index = CatmullRomSplineIndex(keypoint_values_.data(), keypoint_times_.data(),
                                            keypoint_values_.size(), time);
    vector_t result;
    CatmullRomSplineEvaluate(keypoint_values_.data() + index, keypoint_times_.data() + index,
                             time, &result);
    return result;
  }

  // Computes spline position and tangent direction at the given curve time.
  Matrix<K, N, 2> sampleFrame(K time) const {
    const int index = CatmullRomSplineIndex(keypoint_values_.data(), keypoint_times_.data(),
                                            keypoint_values_.size(), time);
    vector_t position, tangent;
    CatmullRomSplineEvaluate(keypoint_values_.data() + index, keypoint_times_.data() + index,
                             time, &position, &tangent);
    Matrix<K, N, 2> result;
    result.col(0) = position;
    // For certain degenerate splines the path derivative and thus the computed tangent direction
    // can be zero.
    const K tangent_length = tangent.norm();
    if (!IsAlmostZero(tangent_length)) {
      result.col(1) = tangent / tangent_length;
    } else {
      result.col(1) = Vector<K, N>::Zero();
    }
    return result;
  }

  // Returns curve time and value for the i-th keypoint. This includes first and last keypoint.
  std::pair<K, vector_t> keypoint(size_t index) const {
    return { keypoint_times_[index], keypoint_values_[index] };
  }

  // Samples the spline for curve times between `t0` and `t1` with a step size of approximately
  // `dt`. The actual step size will be computed as: (t1 - t0) / ceil((t1 - t0) / dt). Always at
  // least two samples will be created.
  bool sample(double t0, double t1, double dt, std::vector<vector_t>& result) {
    ASSERT(t0 < t1, "Invalid range");
    ASSERT(!IsAlmostZero(dt), "Invalid step");
    const int steps = std::max(static_cast<int>(std::round((t1 - t0) / dt)), 2);
    return sample(t0, t1, steps, result);
  }

  // Samples the spline for curve times between `t0` and `t1` with the given number of steps. At
  // least two samples need to be taken.
  bool sample(double t0, double t1, int steps, std::vector<vector_t>& result) {
    ASSERT(t0 < t1, "Invalid range");
    ASSERT(steps > 1, "Invalid step count");
    const double dt = (t1 - t0) / static_cast<double>(steps - 1);
    result.resize(steps);
    for (size_t i = 0; i < result.size(); i++) {
      result[i] = sample(t0);
      t0 += dt;
    }
    return true;
  }

 private:
  std::vector<K> keypoint_times_;
  std::vector<vector_t> keypoint_values_;
  K knot_;
};

using CatmullRomSpline2d = CatmullRomSpline<double, 2>;

// Samples a 3D spline with an offset
template <typename K, typename Derived>
typename std::enable_if<Derived::RowsAtCompileTime == 3, Vector3<K>>::type
SampleWithOffset(const CatmullRomSpline<K, 3>& spline, K time, const Vector3<K>& up,
                 const Eigen::MatrixBase<Derived>& offset) {
  const Matrix<K, 3, 2> frame = spline.sampleFrame(time);
  // frame: 0: position, 1: forward; forward x left = up
  return frame.col(0) + offset[0]*frame.col(1) + offset[1]*up.cross(frame.col(1)) + offset[2]*up;
}

// Samples a 2D spline with an offset
template <typename K, typename Derived>
typename std::enable_if<Derived::RowsAtCompileTime == 2, Vector2<K>>::type
SampleWithOffset(const CatmullRomSpline<K, 2>& spline, K time,
                 const Eigen::MatrixBase<Derived>& offset, Vector2<K>* forward = nullptr,
                 Vector2<K>* sidewards = nullptr) {
  const Matrix<K, 2, 2> frame = spline.sampleFrame(time);
  const Vector2<K> normal = Vector2<K>{-frame(1, 1), frame(0, 1)};
  if (forward != nullptr) {
    *forward = frame.col(1);
  }
  if (sidewards != nullptr) {
    *sidewards = normal;
  }
  return frame.col(0) + offset[0]*frame.col(1) + offset[1]*normal;
}

}  // namespace isaac
