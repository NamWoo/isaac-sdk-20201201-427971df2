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

#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace kinematics {

// Convert from "kinematic" JSON format to "named SOP" JSON format (described in
// engine/gems/sight/sop.hpp). The "kinematic" JSON format used the following structure:
//
//    {
//      "name_of_first_frame": {
//        "frame_id": "example_frame_id",
//        "pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 40.0],
//        "renderables": {
//          "name_of_first_renderable": {
//            "type": "cube",
//            "center": [0.0, 0.0, 0.0],
//            "dimensions": [5.0, 5.0, 5.0]
//          }
//          "name_of_second_renderable": {
//            "type": "sphere",
//            "center": [10.0, 10.0, 10.0],
//            "radius": 30.0
//          }
//        }
//      },
//      ...
//      "name_of_last_frame": {
//        "frame_id": "another_example_frame_id",
//        "renderables": {
//          "name_of_first_renderable": {
//            "type": "line",
//            "point_1": [0.0, 0.0, 0.0],
//            "point_2": [10.0, 20.0, 30.0]
//          }
//          "name_of_second_renderable": {
//            "type": "asset",
//            "name": "my_favorite_robot_mesh"
//          }
//        }
//      }
//    }
//
// Frames:
//   The "kinematic" JSON contains a list of named frames. Each frame can have a "frame_id" and/or a
//   "pose" associated with it. A "frame_id" is a string that corresponds to a frame name in the
//   PoseTree in PoseBackend. A pose is an array of seven numbers representing the orientation and
//   translation of the frame. The first four numbers are quaternion components for orientation and
//   the last three numbers are the XYZ translation: [qw, qx, qy, qz, tx, ty, tz]. If a "frame_id"
//   and "pose" are both included, the transform corresponding to the named "frame_id" is applied
//   prior to the numerically defined "pose." If neither is provided, the frame will not be included
//   in the conversion to "named SOP" format.
//
// Renderables:
//  Each frame contains a list of "renderables" (named geometric primitives to be rendered). If a
//  frame does not include any valid "renderables", the frame will not be included in the conversion
//  to "named SOP" format.
//
// Renderable Geometry:
//   Each "renderable" must contain a valid "type" and enough information to render an object of
//   that type. Currently supported types are: "line_segment", "sphere", "cube" and "asset". To see
//   the required fields for each type, see engine/gems/serialization/json_formatter.hpp. Mapping
//   from type names to Isaac types is as follows:
//
//   "line_segment" <-> geometry::LineSegment3d
//   "sphere"       <-> geometry::SphereD
//   "cube"         <-> geometry::BoxD
//   "asset"        <-> sight::SopAsset
//
// Renderable Transform:
//   In addition to geometric definitions, each "renderable" may optionally include a "frame_id"
//   and/or "pose". If provided these are applied after the frame transform.
//
// Aesthetics:
//   Both "frames" and "renderables" can also include aesthetic attributes. Currently supported
//   aesthetic attributes are "color", "alpha" and "fill_mode". Aesthetic attributes applied to a
//   "frame" will be overridden if the same attribute is defined for a child "renderable".
//
// Color:
//   The "color" can be provided as string or an array of pixel values. If using a string, the
//   string should be a valid Javascript format for an RGB color definition (e.g. '#abc',
//   '#123456', 'rgb(0, 1, 2)). If providing an array of numbers, each component should be an
//   integer value [0, 255]. If three numbers are provided, they will be converted to an RGB color.
//   If four numbers are provided, an additional "alpha" attribute will be calculated from the
//   fourth value (with zero corresponding to fully transparent and 255 corresponding to fully
//   opaque).
//
// Alpha:
//   An "alpha" value is provided as a numeric value from [0, 1]. If an RGBA color and an "alpha"
//   are both provided, the "alpha" attribute will override the "A" component of the RGBA color.
//
// Fill Mode:
//   The "fill_mode" takes a string as input and accepts either "filled" (to render a solid
//   object) or "wireframe" (to render the object as a wireframe).
//
// Examples:
//   An example of the "kinematic" JSON format is included in:
//     engine/gems/kinematics/tests/test_data/sample.kinematics.json
//   The conversion of "sample.kinematic.json" to a "named SOP" JSON file is included in:
//     engine/gems/kinematics/tests/test_data/sample.named_sop.json
Json FromKinematicJson(const Json& json);

// Loads JSON from file and converts from "kinematic" JSON format to "named SOP" JSON format. See
// documentation of "FromKinematicJSON()" for detailed description of supported "kinematic" JSON
// format.
Json FromKinematicJsonFile(const std::string& filename);

}  // namespace kinematics
}  // namespace isaac
