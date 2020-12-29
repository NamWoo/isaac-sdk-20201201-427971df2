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
#include <string>
#include <utility>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/interpolation/interpolation.hpp"
#include "packages/map/gems/obstacle.hpp"

namespace isaac {
namespace map {

// A small distance map which stores dense obstacle information about a small rectangular area.
// A bicubic interpolation is going to be used inside pixels to approximate the distance the best
// way possible while keeping the distance function smooth with smooth derivative.
class DistanceMapObstacle : public Obstacle {
 public:
  // List of parameters relatove to this obstacle
  struct Parameters {
    // Size of the cell of the distance map.
    double cell_size = 0.1;
    // If the value is set, the distance on the edge and outside the image will be controlled by
    // this parameter.
    std::optional<double> outside_distance = std::nullopt;
  };

  DistanceMapObstacle(Image1d&& distance_image)
      : DistanceMapObstacle(std::move(distance_image), Parameters()) {}

  DistanceMapObstacle(Image1d&& distance_image, double cell_size)
      : DistanceMapObstacle(std::move(distance_image), Parameters{cell_size, std::nullopt}) {}

  DistanceMapObstacle(Image1d&& distance_image, Parameters parameters);

  double distance(const Vector2d& point) const override;

  Vector2d gradient(const Vector2d& point) const override;

  Matrix2d hessian(const Vector2d& point) const override;

  void batchDistance(const std::vector<Vector2d>& points,
                     std::vector<double>& distances) const override;

  void batchGradient(const std::vector<Vector2d>& points,
                     std::vector<Vector2d>& gradients) const override;

  void batchHessian(const std::vector<Vector2d>& points,
                     std::vector<Matrix2d>& hessians) const override;

  // Returns a const view of the distance image
  ImageConstView1d distance_image() const {
    return distance_image_.const_view();
  }

  // Returns a view of the image
  ImageView1d distance_image() {
    return distance_image_.view();
  }

  // Returns the cell size
  double cell_size() const {
    return parameters_.cell_size;
  }

 private:
  // Parameters relative to the map
  Parameters parameters_;
  // An occupancy map containing the distance to the nearest blocked grid cell.
  Image1d distance_image_;
  // The inverse of the size of a grid cell.
  double cell_size_inv_;
  // The squared of the inverse of the size of a grid cell.
  double squared_cell_size_inv_;
  // Helper to compute smooth distance.
  std::unique_ptr<BicubicApproximatedFunction<double>> distance_bicubic_;
};

}  // namespace map
}  // namespace isaac
