/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "utils.hpp"

#include <utility>
#include <vector>

#include "engine/core/math/pose2.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/geometry/line_segment.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/serialization/json_formatter.hpp"
#include "engine/gems/sight/sop.hpp"
#include "engine/gems/sight/sop_style.hpp"

namespace isaac {
namespace sight {

void Visualize(const std::vector<Pose2d>& path, std::optional<SopStyle> path_style,
               std::optional<SopStyle> x_axes_style, std::optional<SopStyle> y_axes_style,
               double axes_length, int path_skip, int frame_skip, sight::Sop& sop) {
  // path
  if (path_style) {
    const size_t increment = (path_skip <= 0 ? 1 : path_skip + 1);
    std::vector<Vector2d> points;
    points.reserve(path.size());
    for (size_t i = 0; i < path.size(); i += increment) {
      points.push_back(path[i].translation);
    }
    sop.add([&](sight::Sop& sub_sop) {
      sub_sop.style = *path_style;
      sub_sop.add(points);
    });
  }

  // x axes
  if (x_axes_style) {
    sop.add([&](sight::Sop& sub_sop) {
      sub_sop.style = *x_axes_style;
      const size_t increment = (frame_skip <= 0 ? 1 : frame_skip + 1);
      for (size_t i = 0; i < path.size(); i += increment) {
        sub_sop.add(geometry::LineSegment2d::FromPoints(
            path[i].translation, path[i] * Vector2d{axes_length, 0.0}));
      }
    });
  }

  // y axes
  if (y_axes_style) {
    sop.add([&](sight::Sop& sub_sop) {
      sub_sop.style = *y_axes_style;
      const size_t increment = (frame_skip <= 0 ? 1 : frame_skip + 1);
      for (size_t i = 0; i < path.size(); i += increment) {
        sub_sop.add(geometry::LineSegment2d::FromPoints(
            path[i].translation, path[i] * Vector2d{0.0, axes_length}));
      }
    });
  }
}

void Visualize(const std::vector<Pose2d>& path, std::optional<SopStyle> path_style,
               std::optional<SopStyle> x_axes_style, std::optional<SopStyle> y_axes_style,
               double axes_length, sight::Sop& sop) {
  Visualize(path, std::move(path_style), std::move(x_axes_style), std::move(y_axes_style),
            axes_length, 0, 0, sop);
}

}  // namespace sight
}  // namespace isaac
