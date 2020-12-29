/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/gems/geometry/n_cuboid.hpp"

namespace isaac {

// Bounding box values for a prediction
struct BoundingBoxDetection {
  // Bounding box defined with min and max points in the format
  // {Vector2d(x_min, y_min), Vector2d(x_max, y_max)}
  geometry::RectangleD bounding_box;
  // Class name of the detection
  std::string class_name;
  // Likelihood of the detection being of a specific class
  double probability;
};
}  // namespace isaac
