/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>

#include "engine/gems/sight/kinematics_json.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace kinematics {

const std::string data_path = "engine/gems/sight/tests/test_data/";

TEST(KinematicsJson, StandardConversion) {
  Json kinematics_json = serialization::LoadJsonFromFile(data_path + "sample.kinematics.json");
  Json converted_json = FromKinematicJson(kinematics_json);
  Json expected_json = serialization::LoadJsonFromFile(data_path + "sample.named_sop.json");
  EXPECT_TRUE(converted_json == expected_json);
}

TEST(KinematicsJson, FrameTransforms) {
  Json kinematics_json =
    serialization::LoadJsonFromFile(data_path + "frame_transforms.kinematics.json");
  Json converted_json = FromKinematicJson(kinematics_json);
  Json expected_json =
    serialization::LoadJsonFromFile(data_path + "frame_transforms.named_sop.json");

  // Expected results:
  // - "frame_0" is specified by ONLY a numeric pose.
  // - "frame_1" is specified by ONLY a named frame.
  // - "frame_2" is specified by BOTH a named frame and a numeric pose (applied in that order).
  // - "frame_3" is omitted because it doesn't include "pose" or "frame_id".
  // - "frame_4" is omitted because its provided "pose" was invalid (array length must be 7).
  // - "frame_5" is omitted because its provided "frame_id" is not a string.
  EXPECT_TRUE(converted_json == expected_json);
}

TEST(KinematicsJson, Renderables) {
  Json kinematics_json = serialization::LoadJsonFromFile(data_path + "renderables.kinematics.json");
  Json converted_json = FromKinematicJson(kinematics_json);
  Json expected_json = serialization::LoadJsonFromFile(data_path + "renderables.named_sop.json");

  // Expected results:
  // - "frame_0" contains a valid renderable and will be converted.
  // - "frame_1" is omited because the one renderable it contains is invalid (not enough data
  //   provided to fully define cube).
  // - "frame_2" because there are no renderables included in "renderables".
  // - "frame_3" is omitted because it does not contain a key named "renderables".
  EXPECT_TRUE(converted_json == expected_json);
}

TEST(KinematicsJson, Aesthetics) {
  Json kinematics_json = serialization::LoadJsonFromFile(data_path + "aesthetics.kinematics.json");
  Json converted_json = FromKinematicJson(kinematics_json);
  Json expected_json = serialization::LoadJsonFromFile(data_path + "aesthetics.named_sop.json");

  // Expected results:
  // - "frame_0" contains a style block with default color and fill for frame. Alpha is omitted
  //   because alpha because the input is a string.
  // - Inside "frame_0":
  //  - "renderable_0" has a style block with alpha, color and fill defined. Note: the alpha value
  //     here came from the fourth component of the Pixel4ub input to color.
  //  - "renderable_1" has a style block with alpha, color and fill defined. Note: the alpha value
  //     here comes from the "alpha" input which overrides the alpha component of the Pixel4ub input
  //     to color.
  //  - "renderable_2" omits color because five components were given rather than 3 (RGB) or 4
  //    (RGBA)
  //  - "renderable_3" omits color because components are outside valid range [0, 255]
  //  - "renderable_4" omits alpha because it is outside valid range [0, 1]
  //  - "renderable_5" omits alpha because the input is a string
  //  - "renderable_6" omits fill_mode because the input is a bool
  //  - "renderable_7" omits fill_mode because the input is a numeric value
  //  - "renderable_8" omits fill_mode because the input is a string
  EXPECT_TRUE(converted_json == expected_json);
}

TEST(KinematicsJson, Primitives) {
  Json kinematics_json = serialization::LoadJsonFromFile(data_path + "primitives.kinematics.json");
  Json converted_json = FromKinematicJson(kinematics_json);
  Json expected_json = serialization::LoadJsonFromFile(data_path + "primitives.named_sop.json");

  // Expected results:
  // - Inside "frame_0":
  //  - "renderable_0" is converted to a "cube".
  //  - "renderable_1" is converted to a "line_segment" (a.k.a "line").
  //  - "renderable_2" is converted to a "sphere" (a.k.a. "sphr")
  //  - "renderable_3" is converted to a "asset"
  //  - "renderable_4" is omitted because it does not contain enough data (e.g., center and
  //     dimensions) to convert to a cube
  //  - "renderable_5" is omitted because it does not list a type.
  //  - "renderable_6" is omitted because "icosahedron" is not a supported type.
  //  - "renderable_7" is omitted because the radius is given in an invalid format. Serialization of
  //  - an NSphere expects "radius" to be a scalar value, but input was an array of length one.
  EXPECT_TRUE(converted_json == expected_json);
}

TEST(KinematicsJson, RenderableTransforms) {
  Json kinematics_json =
    serialization::LoadJsonFromFile(data_path + "renderable_transforms.kinematics.json");
  Json converted_json = FromKinematicJson(kinematics_json);
  Json expected_json =
    serialization::LoadJsonFromFile(data_path + "renderable_transforms.named_sop.json");

  // Expected results:
  // - "cube_renderable" in "frame_0" is specified by numeric pose at both the frame AND renderable
  //   level (applied in that order).
  // - "cube_renderable" in "frame_1" is specified by named reference frame at the frame level AND
  //   a numeric pose at the renderable level (applied in that order).
  // - "cube_renderable" in "frame_2" is specified by a named reference frame at the frame level, a
  //   numeric pose at the frame level, a named reference frame at the renderable level AND a
  //   numeric pose at the renderable level (applied in that order).
  EXPECT_TRUE(converted_json == expected_json);
}

}  // namespace kinematics
}  // namespace isaac
