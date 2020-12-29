/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

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

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/polygon.hpp"

namespace isaac {
namespace geometry {

// Computes the convex hull of a a list of points using the Graham scan algorithm.
// Note: Take the list of points by value as it will get modified. This allow using std::move.
template <typename K>
Polygon2<K> ConvexHull(std::vector<Vector2<K>> points) {
  if (points.size() <= 2) return {points};
  // Select a point on the convex hull, the top left most point is an easy candidate.
  size_t min_id = 0;
  for (size_t idx = 1; idx < points.size(); idx++) {
    if (points[idx].x() < points[min_id].x() ||
        (points[idx].x() == points[min_id].x() && (points[idx].y() < points[min_id].y()))) {
      min_id = idx;
    }
  }
  std::swap(points[min_id], points[0]);
  // Sort the point by angle
  std::sort(points.begin() + 1, points.end(),
            [origin = points[0]](const Vector2<K>& a, const Vector2<K>& b) {
    const Vector2<K> da = a - origin;
    const Vector2<K> db = b - origin;
    const K cross_prod = da.y() * db.x() - da.x() * db.y();
    if (cross_prod == K(0)) {
      return da.squaredNorm() < db.squaredNorm();
    }
    return cross_prod < 0;
  });
  // Add the first point at the end
  Polygon2<K> polygon;
  for (const auto& pt : points) {
    while (polygon.points.size() >= 2) {
      const Vector2<K> da = polygon.points.back() - polygon.points[polygon.points.size() - 2];
      const Vector2<K> db = polygon.points.back() - pt;
      const K cross_prod = da.y() * db.x() - da.x() * db.y();
      if (cross_prod > 0) break;
      polygon.points.pop_back();
    }
    polygon.points.push_back(pt);
  }
  return polygon;
}

}  // namespace geometry
}  // namespace isaac
