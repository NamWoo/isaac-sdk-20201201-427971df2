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
#include <limits>
#include <type_traits>

#include "engine/core/math/types.hpp"

namespace isaac {

// Evaluates a bezier spline through points (p0, p1, p2, p3) at spline position `t`.
// The used equation is:
//   B(t) = (1-t)^3 p0 + 3 (1 - t)^2 t P1 + 3 (1-t) t^2 P2 + t^3 P3
template <typename K, int N>
Vector<K, N> Bezier(K t, const Vector<K, N>& p0, const Vector<K, N>& p1, const Vector<K, N>& p2,
                    const Vector<K, N>& p3) {
  const K s = K(1) - t;
  const K ss = s*s;
  const K tt = t*t;
  const K w0 =      ss*s;
  const K w1 = K(3)*ss*t;
  const K w2 = K(3)*s*tt;
  const K w3 =      t*tt;
  return w0*p0 + w1*p1 + w2*p2 + w3*p3;
}

// Iteratively computes the closest point on a Bezier spline to a given point `p`
//
// The algorithm works recursively by first subdividing the spline into `slices` sections and
// finding the closest section. Then it repeats the process for that section to a depth of
// `iterations`. The search is started in the interval [start, end].
//
// This function effectively evaluates slices * iterations samples and the result has a precision
// of (t1 - t0) / slices^iterations.
//
// This function returns the spline position `t` which can be used to compute the actual point.
template <typename K, int N>
K BezierClosestPoint(const Vector<K, N>& p, const Vector<K, N>& p0, const Vector<K, N>& p1,
                     const Vector<K, N>& p2, const Vector<K, N>& p3, unsigned slices,
                     unsigned iterations, K start, K end) {
  if (iterations == 0) return (start + end) / K(2);

  const K tick = (end - start) / K(slices);
  K best_distance = std::numeric_limits<K>::max();
  K t = start;
  K best = t;

  while (t <= end) {
    const Vector<K, N> b = Bezier(t, p0, p1, p2, p3);
    const K current_distance = (b - p).squaredNorm();
    if (current_distance < best_distance) {
      best_distance = current_distance;
      best = t;
    }
    t += tick;
  }

  return BezierClosestPoint(
      p, p0, p1, p2, p3, slices, iterations - 1,
      std::max<K>(best - tick, K(0)), std::min<K>(best + tick, K(1)));
}

// Similar to the other function `BezierClosestPoint`. This overload searches the full range
// [0, 1].
template <typename K, int N>
K BezierClosestPoint(const Vector<K, N>& p, const Vector<K, N>& p0, const Vector<K, N>& p1,
                     const Vector<K, N>& p2, const Vector<K, N>& p3, unsigned slices,
                     unsigned iterations) {
  return BezierClosestPoint(p, p0, p1, p2, p3, slices, iterations, K(0), K(1));
}

// Similar to the other function `BezierClosestPoint`. This overload searches the full range
// [0, 1] and uses reasonable slice and iteration parameters depending on the floating point type.
template <typename K, int N>
K BezierClosestPoint(const Vector<K, N>& p, const Vector<K, N>& p0, const Vector<K, N>& p1,
                     const Vector<K, N>& p2, const Vector<K, N>& p3) {
  static_assert(std::is_floating_point<K>::value, "Type must be float, double or long double");
  constexpr unsigned kNumIterations =
      std::is_same<K, float>::value ? 4 : (std::is_same<K, double>::value ? 6 : 8);
  constexpr unsigned kNumSlices = 10;
  return BezierClosestPoint(p, p0, p1, p2, p3, kNumSlices, kNumIterations);
}

}  // namespace isaac
