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
#include <vector>

#include "engine/core/epsilon.hpp"
#include "engine/gems/geometry/n_cuboid.hpp"
#include "packages/ml/gems/bounding_box_detection.hpp"

namespace isaac {

// Returns the overlap interval in 1 dimension betweeen 2 axis aligned overlapping bounding boxes in
// one dimension, where the length of the interval (a,b) is b-a.
// Inputs : Minimum x/y coordinate of First bounding box,
//          Maximum x/y coordinate of First bounding box,
//          Minimum x/y coordinate of Second bounding box,
//          Maximum x/y coordinate of Second bounding box
template <typename K>
K ComputeOverlap1d(K x1_min, K x1_max, K x2_min, K x2_max) {
  return std::max(K(0), std::min(x2_max, x1_max) - std::max(x2_min, x1_min));
}

// Returns the overlap area between 2 axis aligned bounding boxes of N dimensions and datatype K
// Inputs : bounding_box1 - First Bounding box coordinates of type geometry::NCuboid<K, N>
//          bounding_box2 - Second Bounding box coordinates of type geometry::NCuboid<K, N>
// The bounding boxes are represented as minimum and maximum values of the coordinates along each
// axis (geometry::NCuboid<K, N>::FromOppositeCorners)
template <typename K, int N>
K ComputeOverlap(const geometry::NCuboid<K, N>& bounding_box1,
                 const geometry::NCuboid<K, N>& bounding_box2) {
  K overlap_area = K(1);
  for (size_t dim = 0; dim < N; dim++) {
    overlap_area *= ComputeOverlap1d(bounding_box1.min()[dim], bounding_box1.max()[dim],
                                     bounding_box2.min()[dim], bounding_box2.max()[dim]);
  }
  return overlap_area;
}

// Returns the intersection over union of 2 axis aligned bounding boxes
// Intersection over union = Area of overlap / Area of Union
// Inputs : bounding_box1 - First Bounding box coordinates of type geometry::NCuboid<K, N>
//          bounding_box2 - Second Bounding box coordinates of type geometry::NCuboid<K, N>
// The bounding boxes are represented as minimum and maximum values of the coordinates along each
// axis (geometry::NCuboid<K, N>::FromOppositeCorners)
template <typename K, int N>
K IntersectionOverUnion(const geometry::NCuboid<K, N>& bounding_box1,
                        const geometry::NCuboid<K, N>& bounding_box2) {
  K area_overlap = ComputeOverlap(bounding_box1, bounding_box2);
  K area_union = bounding_box1.volume() + bounding_box2.volume() - area_overlap;
  return IsAlmostZero(area_union) ? K(0) : area_overlap / area_union;
}

// NonMaximumSuppression is used to make sure that in object detection, a particular object is
// identified only once. The input bounding boxes are sorted in descending order based on the
// confidence of each bounding box.
// The IntersectionOfUnion is then computed for every 2 bounding boxes and are filtered if the
// intersection of union is greater than the NonMaximumSupression threshold.
// (https://en.wikipedia.org/wiki/Canny_edge_detector#Non-maximum_suppression)
// Inputs : nms_threshold - Non Maximum threshold (0-1)
//          bounding_boxes - List of Bounding Box detections (not passed by reference because
//          bounding box list is sorted based on probability)
std::vector<BoundingBoxDetection> NonMaximumSuppression(
    const double nms_threshold, std::vector<BoundingBoxDetection>& bounding_boxes);

// Upscales the input bounding box from one resolution to another
// Inputs : bounding_box : Bounding Box to be resized
//          input_dimensions : Input width/height resolution of image for which the bounding_boxes
//          were computed
//          output_dimensions : Output width/height resolution of image for which the bounding_boxes
//          were computed
//          minimum_value : Minimum value to clamp the bounding box coordinates ( Default is 0)
geometry::RectangleD ResizeBoundingBox(const geometry::RectangleD& bounding_box,
                                       const Vector2d& input_dimensions,
                                       const Vector2d& output_dimensions,
                                       const double minimum_value = 0);

}  // namespace isaac
