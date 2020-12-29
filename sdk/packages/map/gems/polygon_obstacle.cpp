/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "polygon_obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "engine/core/epsilon.hpp"
#include "engine/gems/geometry/line_utils.hpp"

namespace isaac {
namespace map {

PolygonObstacle::PolygonObstacle(geometry::Polygon2D polygon)
    : PolygonObstacle(std::vector<geometry::Polygon2D>{{std::move(polygon)}}, Parameters()) {}
PolygonObstacle::PolygonObstacle(geometry::Polygon2D polygon, Parameters parameters)
    : PolygonObstacle(std::vector<geometry::Polygon2D>{{std::move(polygon)}}, parameters) {}
PolygonObstacle::PolygonObstacle(std::vector<geometry::Polygon2D> polygons)
    : PolygonObstacle(std::move(polygons), Parameters()) {}
PolygonObstacle::PolygonObstacle(std::vector<geometry::Polygon2D> polygons, Parameters parameters)
    : polygons_(std::move(polygons)), parameters_(parameters) {
  ASSERT(!polygons_.empty() && !polygons_.front().points.empty(), "We need at least one polygon");
  Vector2d min = polygons_.front().points.front();
  Vector2d max = polygons_.front().points.front();
  for (const auto& polygon : polygons_) {
    ASSERT(!polygon.points.empty(), "Empty polygon are not valid obstacle");
    for (size_t pt = 0; pt < polygon.points.size(); pt++) {
      min.x() = std::min(min.x(), polygon.points[pt].x());
      min.y() = std::min(min.y(), polygon.points[pt].y());
      max.x() = std::max(max.x(), polygon.points[pt].x());
      max.y() = std::max(max.y(), polygon.points[pt].y());
    }
  }
  cell_size_inv_ = 1.0 / parameters_.cell_size;
  squared_cell_size_inv_ = cell_size_inv_ * cell_size_inv_;
  const Vector2d delta = (max - min);
  const double outside_margin = parameters_.max_distance + 4.0 * parameters_.cell_size;
  const int rows =
      static_cast<int>(std::ceil((delta.x() + 2 * outside_margin) / parameters_.cell_size));
  const int cols =
      static_cast<int>(std::ceil((delta.y() + 2 * outside_margin) / parameters_.cell_size));

  offset_ = Vector2d(min.x() - outside_margin, min.y() - outside_margin);

  distance_ = std::make_unique<BicubicApproximatedFunction<double>>(
      [this](size_t row, size_t col) {
        const Vector2d position(row * parameters_.cell_size + offset_.x(),
                                col * parameters_.cell_size + offset_.y());
        // Capping the value to make sure that outside the range we have a flat distance.
        double min_distance = parameters_.max_distance;
        for (const auto& polygon : polygons_) {
          min_distance = std::min(min_distance, polygon.signedDistance(position));
        }
        return min_distance;
      }, rows, cols);
}

double PolygonObstacle::distance(const Vector2d& point) const {
  const Vector2d pos = (point - offset_) * cell_size_inv_;
  return distance_->get(pos.x(), pos.y());
}

Vector2d PolygonObstacle::gradient(const Vector2d& point) const {
  const Vector2d pos = (point - offset_) * cell_size_inv_;
  return distance_->gradient(pos.x(), pos.y()) * cell_size_inv_;
}

Matrix2d PolygonObstacle::hessian(const Vector2d& point) const {
  const Vector2d pos = (point - offset_) * cell_size_inv_;
  return distance_->hessian(pos.x(), pos.y()) * squared_cell_size_inv_;
}

void PolygonObstacle::batchDistance(const std::vector<Vector2d>& points,
                                    std::vector<double>& distances) const {
  distances.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    distances[idx] = distance(points[idx]);
  }
}

void PolygonObstacle::batchGradient(const std::vector<Vector2d>& points,
                                    std::vector<Vector2d>& gradients) const {
  gradients.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    gradients[idx] = gradient(points[idx]);
  }
}

void PolygonObstacle::batchHessian(const std::vector<Vector2d>& points,
                                    std::vector<Matrix2d>& hessians) const {
  hessians.resize(points.size());
  // TODO(ben): provide GPU accelerated implementation.
  for (size_t idx = 0; idx < points.size(); idx++) {
    hessians[idx] = hessian(points[idx]);
  }
}

}  // namespace map
}  // namespace isaac
