/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/gems/geometry/types.hpp"
#include "engine/gems/interpolation/interpolation.hpp"
#include "packages/map/gems/obstacle.hpp"

namespace isaac {
namespace map {

// A simple Polygon obstacle at a certain position (no self intersection).
// The distance is 0 on the boundary of the object and negative inside.
// In order to produce a smooth and differentiable distance function, we compute the exact distance
// at some fixed position on a grid and a bicubic interpolation is used between these points.
// The distance is capped to a maximum value.
class PolygonObstacle : public Obstacle {
 public:
  // List of parameters relatove to this obstacle
  struct Parameters {
    // Size of the cell of the distance bicubic interpolation.
    double cell_size = 0.1;
    // Maximum distance to the obstacle, passed this distance, distance will be capped at this value
    // (in practice the returned value might be slightly higher).
    double max_distance = 1.5;
  };

  PolygonObstacle(std::vector<geometry::Polygon2D> polygons, Parameters parameters);
  PolygonObstacle(std::vector<geometry::Polygon2D> polygons);
  PolygonObstacle(geometry::Polygon2D polygon, Parameters parameters);
  PolygonObstacle(geometry::Polygon2D polygon);

  double distance(const Vector2d& point) const override;

  Vector2d gradient(const Vector2d& point) const override;

  Matrix2d hessian(const Vector2d& point) const override;

  void batchDistance(const std::vector<Vector2d>& points,
                     std::vector<double>& distance) const override;

  void batchGradient(const std::vector<Vector2d>& points,
                     std::vector<Vector2d>& gradients) const override;

  void batchHessian(const std::vector<Vector2d>& points,
                     std::vector<Matrix2d>& hessians) const override;

 private:
  std::vector<geometry::Polygon2D> polygons_;
  Parameters parameters_;
  std::unique_ptr<BicubicApproximatedFunction<double>> distance_;
  double cell_size_inv_;
  double squared_cell_size_inv_;
  Vector2d offset_;
};

}  // namespace map
}  // namespace isaac
