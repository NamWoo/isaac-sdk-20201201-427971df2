/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/math/pose2.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/sight/sop.hpp"
#include "engine/gems/sight/sop_style.hpp"

namespace isaac {
namespace sight {

// Shows a list of poses with coordinate frame indicators. If optional parameters are not set the
// corresponding pieces will not be visualized. If `path_skip` is set to a value greater than 0
// only some poses will be visualized. In particular `path_skip` == 1 will show only every
// other point. Similarly for `frame_skip` for rendering the coordinate frame.
void Visualize(const std::vector<Pose2d>& path, std::optional<SopStyle> path_style,
               std::optional<SopStyle> x_axes_style, std::optional<SopStyle> y_axes_style,
               double axes_length, int path_skip, int frame_skip, sight::Sop& sop);

// Similar to `Visualize` but with `path_skip` and `frame_skip` set to 0.
void Visualize(const std::vector<Pose2d>& path, std::optional<SopStyle> path_style,
               std::optional<SopStyle> x_axes_style, std::optional<SopStyle> y_axes_style,
               double axes_length, sight::Sop& sop);

}  // namespace sight
}  // namespace isaac
