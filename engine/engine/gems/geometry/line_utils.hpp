/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cmath>
#include <type_traits>

#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/gems/geometry/polyline.hpp"

namespace isaac {
namespace geometry {

// Returns the closest point of a segment (ab) to a point `p`.
// NOTE: It can be used with Line/HalfLine/Segment.
template<typename Line>
typename Line::Vector_t ClosestPointToLine(const Line& line, const typename Line::Vector_t& p);

// Returns the squared distance between a point and a Line/HalfLine/Segment.
// Avoid any sqrt by using the pytagorian theorem.
template<typename Line>
typename Line::Scalar SquaredDistancePointToLine(const Line& line,
                                                 const typename Line::Vector_t& p);

// Returns the distance between a point and a Line/HalfLine/Segment.
template<typename Line>
typename Line::Scalar DistancePointToLine(const Line& line, const typename Line::Vector_t& p);

// Returns whether two Line/HalfLine/Segment are intersecting.
// In addition if the line interesect lamda_{A/B} would be filled such as:
// A.origin() + lamda_a A.direction() == B.origin() + lamda_b B.direction()
template<typename LineT1, typename LineT2>
bool AreLinesIntersecting(const LineT1& line_a, const LineT2& line_b,
                          typename LineT1::Scalar* lamda_a = nullptr,
                          typename LineT1::Scalar* lamda_b = nullptr);

// Returns the closest point of a segment [ab] to a point `p`.
template<typename K, int N>
Vector<K, N> ClosestPointToSegement(const Vector<K, N>& a, const Vector<K, N>& b,
                                    const Vector<K, N>& p);

// Returns the closest point of a polyline [ab...yz] to a point `p`.
// If `segment_index` is provided, it will contain the index of the segment the closest to the
// point.
template<typename K, int N>
Vector<K, N> ClosestPointToPolyline(const Polyline<K, N>& polyline, const Vector<K, N>& p,
                                    size_t* segment_index = nullptr);

// Returns the squared distance of a point `p` to a segment [ab].
template<typename K, int N>
K DistanceSquaredPointToSegement(const Vector<K, N>& a, const Vector<K, N>& b,
                                 const Vector<K, N>& p);

// Returns the distance of a point `p` to a segment [ab].
template<typename K, int N>
K DistancePointToSegement(const Vector<K, N>& a, const Vector<K, N>& b, const Vector<K, N>& p);

// -------------------------------------------------------------------------------------------------

template<typename Line>
typename Line::Vector_t ClosestPointToLine(const Line& line, const typename Line::Vector_t& p) {
  const typename Line::Vector_t& ray = line.direction();
  const typename Line::Vector_t ap = p - line.origin();
  const typename Line::Scalar l = line.clamp(ray.dot(ap) / ray.squaredNorm());
  return line.origin() + l * ray;
}

template<typename Line>
typename Line::Scalar SquaredDistancePointToLine(const Line& line,
                                                 const typename Line::Vector_t& p) {
  using K = typename Line::Scalar;
  const typename Line::Vector_t& ray = line.direction();
  const typename Line::Vector_t ap = p - line.origin();
  const K ray_sq_norm = ray.squaredNorm();
  // Check this is not an empty segement, in which case the distance is the distance to one of the
  // extremities.
  if (IsAlmostZero(ray_sq_norm)) return ap.squaredNorm();
  const K dot = ray.dot(ap);
  const K l = dot / ray_sq_norm;
  const K cl = line.clamp(l);
  const K cl2 = cl * cl * ray_sq_norm;
  return ap.squaredNorm() + cl2 - K(2) * l * cl * ray_sq_norm;
}

template<typename Line>
typename Line::Scalar DistancePointToLine(const Line& line, const typename Line::Vector_t& p) {
  return std::sqrt(SquaredDistancePointToLine(line, p));
}

// P(x, y) is on a line iff (P-A)xR_a = 0
// We are looking for lamda such as (B + lamda R_b - A) x R_a = 0
// lamda = (A-B) x R_a / (R_b x R_a)
// x denotes the cross product here.
template<typename LineT1, typename LineT2>
bool AreLinesIntersecting(const LineT1& line_a, const LineT2& line_b,
                          typename LineT1::Scalar* lamda_a,
                          typename LineT1::Scalar* lamda_b) {
  static_assert(LineT1::kDimension == 2 && LineT2::kDimension == 2,
                "AreLinesIntersecting only works in 2D");
  static_assert(std::is_same<typename LineT1::Scalar, typename LineT2::Scalar>::value,
                "Type missmatch");
  using K = typename LineT1::Scalar;
  K lA, lB;
  if (lamda_a == nullptr) lamda_a = &lA;
  if (lamda_b == nullptr) lamda_b = &lB;
  const Vector2<K>& R_a = line_a.direction();
  const Vector2<K>& R_b = line_b.direction();
  const K ray_cross = CrossProduct(R_b, R_a);
  // The two lines are parallel, they could still interesect in one or an infinity of points.
  if (IsAlmostZero(ray_cross)) {
    const Vector2<K> pt = ClosestPointToLine(line_a, line_b.origin());
    constexpr K kEpsilonThreshold = MachineEpsilon<K> * 1000.0;
    if (SquaredDistancePointToLine(line_b, pt) >= kEpsilonThreshold) return false;
    const K R_a_norm = line_a.direction().squaredNorm();
    const K R_b_norm = line_b.direction().squaredNorm();
    *lamda_a = IsAlmostZero(R_a_norm) ? K(0) : (pt - line_a.origin()).dot(R_a) / R_a_norm;
    *lamda_b = IsAlmostZero(R_b_norm) ? K(0) : (pt - line_b.origin()).dot(R_b) / R_b_norm;
  } else {
    const Vector2<K> BA = line_a.origin() - line_b.origin();
    *lamda_b = CrossProduct(BA, R_a) / ray_cross;
    *lamda_a = CrossProduct(BA, R_b) / ray_cross;
  }
  // Checks that both lamda is within the range of the line.
  return line_a.clamp(*lamda_a) == *lamda_a && line_b.clamp(*lamda_b) == *lamda_b;
}

template<typename K, int N>
Vector<K, N> ClosestPointToSegement(const Vector<K, N>& a, const Vector<K, N>& b,
                                    const Vector<K, N>& p) {
  const Vector<K, N> ab = b - a;
  const Vector<K, N> ap = p - a;
  const K l = ab.dot(ap);
  if (l <= K(0)) {
    return a;
  }
  const K ab_n2 = ab.squaredNorm();
  if (l >= ab_n2) {
    return b;
  }
  return a + (l / ab_n2) * ab;
}

template<typename K, int N>
Vector<K, N> ClosestPointToPolyline(const Polyline<K, N>& polyline, const Vector<K, N>& p,
                                    size_t* segment_index) {
  ASSERT(polyline.size() > 0, "Polyline is empty");
  if (segment_index) *segment_index = 0;
  K best_dist = (p - polyline.front()).squaredNorm();
  Vector<K, N> ret = polyline.front();
  for (size_t i = 1; i < polyline.size(); i++) {
    const auto vec = ClosestPointToSegement(polyline[i - 1], polyline[i], p);
    const K dist = (vec - p).squaredNorm();
    if (dist < best_dist) {
      best_dist = dist;
      ret = vec;
      if (segment_index) *segment_index = i-1;
    }
  }
  return ret;
}

template<typename K, int N>
K DistanceSquaredPointToSegement(const Vector<K, N>& a, const Vector<K, N>& b,
                                 const Vector<K, N>& p) {
  const Vector<K, N> ab = b - a;
  const Vector<K, N> ap = p - a;
  const K l = ab.dot(ap);
  if (l <= K(0)) {
    return ap.squaredNorm();
  }
  const K ab_n2 = ab.squaredNorm();
  if (l >= ab_n2) {
    return (b - p).squaredNorm();
  }
  return ap.squaredNorm() - l * l / ab_n2;
}

template<typename K, int N>
K DistancePointToSegement(const Vector<K, N>& a, const Vector<K, N>& b, const Vector<K, N>& p) {
  return std::sqrt(DistanceSquaredPointToSegement(a, b, p));
}

}  // namespace geometry
}  // namespace isaac
