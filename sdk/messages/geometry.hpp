/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/gems/geometry/n_cuboid.hpp"
#include "engine/gems/geometry/plane.hpp"
#include "engine/gems/geometry/polyline.hpp"
#include "messages/geometry.capnp.h"
#include "messages/math.hpp"

namespace isaac {

inline geometry::PlaneD FromProto(::PlaneProto::Reader reader) {
  return {FromProto(reader.getNormal()), reader.getOffset()};
}

inline void ToProto(const geometry::PlaneD& plane, ::PlaneProto::Builder builder) {
  ToProto(plane.normal(), builder.initNormal());
  builder.setOffset(plane.offset());
}

inline geometry::RectangleD FromProto(::RectangleProto::Reader reader) {
  return geometry::RectangleD::FromOppositeCorners(FromProto(reader.getMin()),
                                                   FromProto(reader.getMax()));
}

inline void ToProto(const geometry::RectangleD& rectangle, ::RectangleProto::Builder builder) {
  ToProto(rectangle.min(), builder.initMin());
  ToProto(rectangle.max(), builder.initMax());
}

inline geometry::BoxD FromProto(::BoxProto::Reader reader) {
  return geometry::BoxD::FromOppositeCorners(FromProto(reader.getMin()),
                                             FromProto(reader.getMax()));
}

inline void ToProto(const geometry::BoxD& box, ::BoxProto::Builder builder) {
  ToProto(box.min(), builder.initMin());
  ToProto(box.max(), builder.initMax());
}

inline geometry::Polyline2d FromProto(::Polyline2dProto::Reader reader) {
  geometry::Polyline2d polyline;
  polyline.reserve(reader.getLine().size());
  for (::Vector2dProto::Reader point : reader.getLine()) {
    polyline.push_back(FromProto(point));
  }
  return polyline;
}

inline void ToProto(const geometry::Polyline2d& polyline, ::Polyline2dProto::Builder builder) {
  auto line = builder.initLine(polyline.size());
  size_t index = 0;
  for (auto& point : polyline) {
    ::Vector2dProto::Builder builder = line[index++];
    ToProto(point, builder);
  }
}

}  // namespace isaac
