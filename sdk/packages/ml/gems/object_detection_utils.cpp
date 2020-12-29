/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "packages/ml/gems/object_detection_utils.hpp"

#include <algorithm>
#include <vector>

#include "engine/core/math/utils.hpp"
#include "engine/gems/geometry/n_cuboid.hpp"
#include "packages/ml/gems/bounding_box_detection.hpp"

namespace isaac {

std::vector<BoundingBoxDetection> NonMaximumSuppression(
    const double nms_threshold, std::vector<BoundingBoxDetection>& bounding_boxes) {
  std::stable_sort(bounding_boxes.begin(), bounding_boxes.end(),
                   [](const BoundingBoxDetection& box1, const BoundingBoxDetection& box2) {
                     return box1.probability > box2.probability;
                   });
  std::vector<BoundingBoxDetection> filtered_bounding_boxes;
  for (const auto& box1 : bounding_boxes) {
    bool retain_bounding_box = true;
    for (const auto& box2 : filtered_bounding_boxes) {
      if (retain_bounding_box) {
        double overlap = IntersectionOverUnion(box1.bounding_box, box2.bounding_box);
        retain_bounding_box = overlap <= nms_threshold;
      } else {
        break;
      }
    }
    if (retain_bounding_box) {
      filtered_bounding_boxes.push_back(box1);
    }
  }
  return filtered_bounding_boxes;
}

geometry::RectangleD ResizeBoundingBox(const geometry::RectangleD& input_bounding_box,
                                       const Vector2d& input_dimensions,
                                       const Vector2d& output_dimensions, double minimum_value) {
  geometry::RectangleD output_bounding_box;
  ASSERT(!IsAlmostZero(input_dimensions[0]) && !IsAlmostZero(input_dimensions[1]),
         "Input dimension should be greater than 0");
  ASSERT(!IsAlmostZero(output_dimensions[0]) && !IsAlmostZero(output_dimensions[1]),
         "Output dimension should be greater than 0");

  const Eigen::Array2d scaling_array = input_dimensions.array() / output_dimensions.array();
  const double scaling_factor = std::min(scaling_array[0], scaling_array[1]);
  const Eigen::Array2d correction =
      (input_dimensions.array() - scaling_factor * output_dimensions.array()) * 0.5;

  output_bounding_box.min() = input_bounding_box.min().array() - correction;
  output_bounding_box.max() = input_bounding_box.max().array() - correction;

  // Restore to input resolution
  output_bounding_box.min().array() /= scaling_factor;
  output_bounding_box.max().array() /= scaling_factor;

  // Clamp the computed coordinates to minimum_value provided and the image dimensions
  output_bounding_box.min().x() =
      Clamp(output_bounding_box.min().x(), minimum_value, output_dimensions[0]);
  output_bounding_box.max().x() =
      Clamp(output_bounding_box.max().x(), minimum_value, output_dimensions[0]);
  output_bounding_box.min().y() =
      Clamp(output_bounding_box.min().y(), minimum_value, output_dimensions[1]);
  output_bounding_box.max().y() =
      Clamp(output_bounding_box.max().y(), minimum_value, output_dimensions[1]);
  return output_bounding_box;
}

}  // namespace isaac
