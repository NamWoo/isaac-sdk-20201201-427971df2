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
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/line_segment.hpp"
#include "engine/gems/geometry/line_utils.hpp"

namespace isaac {
namespace geometry {

// A polygon in 2D, the last point and first point are connected
template <typename K>
struct Polygon2 {
  // Type of a point.
  using Vector_t = Vector2<K>;
  using Scalar = K;

  std::vector<Vector_t> points;

  // Returns whether a point is inside the polygon
  // Note that this method is not accurate if the point lies too close to one edge, Especially if
  // the point is really close to the edge or of a vertex, it might return true.
  // This function used the strategy of counting the intersection between the segments of the
  // polygon and a virtual segment from the `pt` to a point outside of the polygon. While fast, this
  // method does not work well when the virtual segment is too close to one of the vertices. In this
  // case it fallbacks to the slower implementation below.
  bool isInside(const Vector_t& pt) const {
    if (points.empty()) return false;
    // Select a point outside of the polygon (max(x) + 1, max(y) + 2).
    // Picking a different constant for x and y helps reduce the chance the point is aligned with
    // one vertex.
    Vector_t max = points.front();
    for (const auto& point : points) {
      max.x() = std::max(max.x(), point.x() + K(1));
      max.y() = std::max(max.y(), point.y() + K(2));
    }
    // If the maximum value of x+1 is less or equal to our point, then we are obviously outside.
    if (max.x() <= pt.x()) return false;
    if (max.y() <= pt.y()) return false;
    const LineSegment<K, 2> seg(pt, max);
    // Count how many segment it intersects, if it's an odd number then the point is inside or on
    // one edge.
    int counter = 0;
    for (size_t ix = 0; ix < points.size(); ix++) {
      const LineSegment<K, 2> edge(points[ix], points[(ix + 1) % points.size()]);
      K lambda;
      if (AreLinesIntersecting(seg, edge, nullptr, &lambda)) {
        counter++;
        // If our lines are too close, we fallback to the slow but accurate method.
        if (IsAlmostZero(lambda) || IsAlmostOne(lambda)) {
          return slowIsInside(pt);
        }
      }
    }
    return counter % 2 == 1;
  }

  // Returns whether a point is inside the polygon
  // This function is a bit slow and should not be called in critical path too often.
  // This function used the strategy to measure the sum of the field of view the segment are
  // observed. It was the most accurate solution, however it requires using trigonometric functions,
  // which are quite slow.
  bool slowIsInside(const Vector_t& pt) const {
    if (points.empty()) return false;
    K sum_angle = K(0);
    Vector_t delta = points.back() - pt;
    // If delta == 0, it means pt is the same as one of the vertex.
    if (IsAlmostZero(delta.squaredNorm())) return true;
    K last_angle = std::atan2(delta.y(), delta.x());
    for (size_t ix = 0; ix < points.size(); ix++) {
      delta = points[ix] - pt;
      // If delta == 0, it means pt is the same as one of the vertex.
      if (IsAlmostZero(delta.squaredNorm())) return true;
      const K angle = std::atan2(delta.y(), delta.x());
      const K delta_angle = DeltaAngle(angle, last_angle);
      // If delta == +/- pi, it means pt is on one of the segment.
      if (IsAlmostEqualRelative(std::abs(delta_angle), Pi<K>)) {
        return true;
      }
      sum_angle += delta_angle;
      last_angle = angle;
    }
    return std::abs(sum_angle) > Pi<K>;
  }

  // Returns the distance of a point from the polygon edges
  K distance(const Vector_t& pt, Vector_t* grad = nullptr) const {
    K squared_dist = std::numeric_limits<K>::max();
    for (size_t ix = 0; ix < points.size(); ix++) {
      const LineSegment<K, 2> edge(points[ix], points[(ix + 1) % points.size()]);
      const Vector_t closest = ClosestPointToLine(edge, pt);
      const K dist = (pt - closest).squaredNorm();
      if (dist < squared_dist) {
        squared_dist = dist;
        if (grad) *grad = pt - closest;
      }
    }
    if (grad) grad->normalize();
    return std::sqrt(squared_dist);
  }

  // Returns the signed distance of a point from the polygon (negative means inside the polygon)
  K signedDistance(const Vector_t& pt, Vector_t* grad = nullptr) const {
    const K dist = distance(pt, grad);
    // If the point is inside the polygon, we need to revert the direction of the gradient.
    if (isInside(pt)) {
      if (grad) *grad = -(*grad);
      return -dist;
    }
    return dist;
  }

  // Casts to a different type
  template <typename S, typename std::enable_if_t<!std::is_same<S, K>::value, int> = 0>
  Polygon2<S> cast() const {
    std::vector<Vector2<S>> pts;
    pts.reserve(points.size());
    for (const auto& pt : points) {
      pts.emplace_back(pt.template cast<S>());
    }
    return Polygon2<S>{std::move(pts)};
  }
  template<typename S, typename std::enable_if_t<std::is_same<S, K>::value, int> = 0>
  const Polygon2& cast() const {
    // Nothing to do as the type does not change
    return *this;
  }
};

// A polygon in 3D, the last point and first point are connected
template <typename K>
struct Polygon3 {
  // Type of a point.
  using Vector_t = Vector3<K>;
  using Scalar = K;

  std::vector<Vector_t> points;
};

using Polygon2D = Polygon2<double>;
using Polygon2F = Polygon2<float>;
using Polygon3D = Polygon3<double>;
using Polygon3F = Polygon3<float>;

}  // namespace geometry
}  // namespace isaac
