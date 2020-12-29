/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "kinematics_json.hpp"

#include <string>

#include "engine/core/logger.hpp"
#include "engine/gems/image/color.hpp"
#include "engine/gems/serialization/json_formatter.hpp"
#include "engine/gems/sight/serialize.hpp"
#include "engine/gems/sight/sop_asset.hpp"

namespace isaac {
namespace kinematics {


namespace kinematic_json_details {

// Check for valid "frame_id" and "pose". If one or both are available add to output_json, update
// final_data_root, and return true. If neither are available, make no updates and return false. If
// both "frame_id" and "pose" are available, "frame_id" is applied prior to "pose".
bool ProcessTransform(const Json& input_json, const std::string& element_identifier,
                      Json& output_json, std::reference_wrapper<Json>& final_data_root) {
  auto frame_id = serialization::TryGetFromMap<std::string>(input_json, "frame_id");
  auto pose = serialization::TryGetFromMap<Pose3d>(input_json, "pose");

  // Log warning if "frame_id" is included, but is invalid
  if (input_json.count("frame_id") && !frame_id) {
    LOG_WARNING("Invalid \"frame_id\" provided for %s. \"%s\" is not a valid \"frame_id\" and will "
                "not be included in conversion.", element_identifier.c_str(),
                input_json["frame_id"].dump().c_str());
  }

  // Log warning if "frame_id" is included, but is invalid
  if (input_json.count("pose") && !pose) {
    LOG_WARNING("Invalid \"pose\" provided for %s. \"%s\" is not a valid \"pose\" and will not be "
                "included in conversion.", element_identifier.c_str(),
                input_json["pose"].dump().c_str());
  }

  if (!frame_id && !pose) {
    return false;
  }

  if (frame_id) {
    // Add named frame
    final_data_root.get()["type"] = "sop";
    final_data_root.get()["pose"]["type"] = "f";
    final_data_root.get()["pose"]["pose"] = *frame_id;

    // If pose is also available, move final_data_root down one level
    if (pose) {
      final_data_root = std::ref(output_json["data"][0]);
    }
  }

  if (pose) {
    // Add numeric pose
    final_data_root.get()["type"] = "sop";
    final_data_root.get()["pose"]["type"] = "3d";
    serialization::Set(final_data_root.get()["pose"]["pose"], *pose);
  }
  return true;
}

// Helper function to convert Pixel4ub to Pixel3ub (by dropping the alpha component)
Pixel3ub ToPixel3ub(const Pixel4ub& pixel) {
  return Pixel3ub({pixel[0], pixel[1], pixel[2]});
}

// Tries to get color from input_json if key "color" exists.
// -- First try to get color as a string. If available, JSON containing the color is returned.
// -- If input_json["color"] cannot be converted to string, try to convert to Pixel4ub. If valid,
//    color is converted to hex string and JSON containing the color is returned. If alpha
//    channel (other than 255) is provided, "alpha" attribute is also added to JSON.
// -- If no valid color found, return std::nullopt.
std::optional<Json> ProcessColor(const Json& input_json, const std::string& element_identifier) {
  if (input_json.count("color")) {
    Json color_json;

    auto color_string = serialization::TryGetFromMap<std::string>(input_json, "color");
    auto color_pixel = serialization::TryGetFromMap<Pixel4ub>(input_json, "color");

    if (color_string) {
      // TODO validate that string is a valid color representation.
      color_json["style"]["color"] = *color_string;
      return color_json;
    } else if (color_pixel) {
      color_json["style"]["color"] = ToHexString(ToPixel3ub(*color_pixel));
      if ((*color_pixel)[3] != 255) {
        color_json["style"]["alpha"] = static_cast<double>((*color_pixel)[3])/255.0;
      }
      return color_json;
    } else {
      LOG_WARNING("Invalid \"color\" provided for %s. \"%s\" is not a valid color and will not "
                  "be included in conversion.", element_identifier.c_str(),
                  input_json["color"].dump().c_str());
      return std::nullopt;
    }
  }
  return std::nullopt;
}

// Tries to get alpha from input_json if key "alpha" exists.
// -- Alpha must be in range [0, 1]. If valid, return JSON containing alpha
// -- If no valid alpha found, return std::nullopt.
std::optional<Json> ProcessAlpha(const Json& input_json, const std::string& element_identifier) {
  if (input_json.count("alpha")) {
    auto alpha = serialization::TryGetFromMap<double>(input_json, "alpha");
    if (alpha) {
      if (*alpha >= 0.0 && *alpha <= 1.0) {
        Json alpha_json;
        alpha_json["style"]["alpha"] = *alpha;
        return alpha_json;
      } else {
        LOG_WARNING("Invalid \"alpha\" provided for %s. Alpha value must be in range [0, 1]. "
                    "Provided alpha \"%f\" is out of range and will not be included in conversion.",
                    element_identifier.c_str(), *alpha);
        return std::nullopt;
      }
    } else {
      LOG_WARNING("Invalid \"alpha\" provided for %s. %s is not a valid alpha and will not "
                  "be included in conversion. Alpha must be a numeric value in range [0, 1].",
                  element_identifier.c_str(), input_json["alpha"].dump().c_str());
      return std::nullopt;
    }
  }
  return std::nullopt;
}

// Tries to get fill mode from input_json if key "fill_mode" exists.
// -- "fill_mode" must be "filled" or "wireframe". If valid, return JSON containing fill mode.
// -- If no valid fill mode found, return std::nullopt.
std::optional<Json> ProcessFillMode(const Json& input_json, const std::string& element_identifier) {
  if (input_json.count("fill_mode")) {
    auto fill_mode = serialization::TryGetFromMap<std::string>(input_json, "fill_mode");
    if (fill_mode) {
      if (*fill_mode == "filled") {
        Json fill_mode_json;
        fill_mode_json["style"]["fill"] = true;
        return fill_mode_json;
      } else if (*fill_mode == "wireframe") {
        Json fill_mode_json;
        fill_mode_json["style"]["fill"] = false;
        return fill_mode_json;
      } else {
        LOG_WARNING("Invalid \"fill_mode\" provided for %s. Fill mode must be \"filled\" or "
                    "\"wireframe\". Provided fill mode \"%s\" is invalid and will not be included "
                    "in conversion.", element_identifier.c_str(), (*fill_mode).c_str());
        return std::nullopt;
      }
    } else {
      LOG_WARNING("Invalid \"fill_mode\" provided for %s. \"%s\" is not a valid fill mode and "
                  "will not be included in conversion. Fill mode must be a string equal to either "
                  "\"filled\" or \"wireframe\".", element_identifier.c_str(),
                  input_json["fill_mode"].dump().c_str());
      return std::nullopt;
    }
  }
  return std::nullopt;
}

// Check for optional attributes "color", "alpha" and "fill_mode". If one ore more available, merge
// attributes and return merged JSON. If no aesthetic attributes are found, return std::nullopt.
std::optional<Json> ProcessAesthetics(const Json& input_json,
                                      const std::string& element_identifier) {
  Json aesthetics_json;

  auto maybe_color = ProcessColor(input_json, element_identifier);
  if (maybe_color) {aesthetics_json = *maybe_color;}

  auto maybe_alpha = ProcessAlpha(input_json, element_identifier);
  if (maybe_alpha) {aesthetics_json = serialization::MergeJson(aesthetics_json, *maybe_alpha);}

  auto maybe_fill = ProcessFillMode(input_json, element_identifier);
  if (maybe_fill) {aesthetics_json = serialization::MergeJson(aesthetics_json, *maybe_fill);}

  if (!aesthetics_json.empty()) {
    return aesthetics_json;
  } else {
    return std::nullopt;
  }
}

// If primitive of type T is available, return JSON with serialized primitive.
// Else, return std::nullopt.
template <typename T>
std::optional<Json> TryGetPrimitive(const std::string& type_name, const std::string& frame_name,
                                    const std::string& renderable_name, const Json& input_json) {
  auto maybe = serialization::TryGet<T>(input_json);
  if (maybe) {
    return sight::ToJson(*maybe);
  }
  LOG_WARNING("Unable to get valid \"%s\" for renderable \"%s\" in frame \"%s\". Renderable \"%s\" "
              "will not be included in conversion.", type_name.c_str(), renderable_name.c_str(),
              frame_name.c_str(), renderable_name.c_str());
  return std::nullopt;
}

// Tries to get primitive from input_json if key "type" exists.
// -- If a valid "type" is provided and a primitive of that type can be created, return JSON with
// -- serialized primitive.
// -- Else, do nothing and return false.
std::optional<Json> ProcessPrimitive(const std::string& frame_name,
                                     const std::string& renderable_name,
                                     const Json& input_json) {
  if (input_json.count("type")) {
    auto type = serialization::TryGetFromMap<std::string>(input_json, "type");
    if (*type == "line_segment") {
      return TryGetPrimitive<geometry::LineSegment3d>(*type, frame_name, renderable_name,
                                                      input_json);
    }  else if (*type == "sphere") {
      return TryGetPrimitive<geometry::SphereD>(*type, frame_name, renderable_name, input_json);
    } else if (*type == "cube") {
      return TryGetPrimitive<geometry::BoxD>(*type, frame_name, renderable_name, input_json);
    } else if (*type == "asset") {
      auto name = input_json.find("name");
      if (name == input_json.end()) return std::nullopt;
      return sight::ToJson(sight::SopAsset::FromName(*name));
    } else {
      LOG_WARNING("Invalid \"type\" for renderable \"%s\" in frame \"%s\". Type must be specified "
                  "as a string identifier for a primitive geometry. %s is not a valid \"type\"."
                  " Supported primitives are \"line_segment\", \"sphere\", \"cube\", and \"asset\"."
                  " Renderable \"%s\" will not be included in conversion.",
                  renderable_name.c_str(), frame_name.c_str(), input_json["type"].dump().c_str(),
                  renderable_name.c_str());
      return std::nullopt;
    }
  } else {
    LOG_WARNING("Could not find \"type\" for renderable \"%s\" in frame \"%s\". "
                "Renderable \"%s\" will not be included in conversion.",
                renderable_name.c_str(), frame_name.c_str(), renderable_name.c_str());
    return std::nullopt;
  }
}

// Process each renderable. To return Json, the renderable must include a valid "type" and required
// data to generate a primitive of that type. If a valid primitive cannot be generate, std::nullopt
// will be returned.
std::optional<Json> ProcessRenderable(const Json& renderable_json,
                                      const std::string& renderable_name,
                                      const std::string& frame_name) {
  Json sop;
  auto final_data_root = std::ref(sop);

  // If (optional) transform is available check for (optional) aesthetic attributes and move
  // final_data_root down one level. Else, move directly to checking for (optional) aesthetic
  // attributes.
  const std::string element_identifier = "renderable \"" + renderable_name +
                                         "\" in frame \"" + frame_name + "\"";
  auto aesthetic_json = ProcessAesthetics(renderable_json, element_identifier);
  if (ProcessTransform(renderable_json, element_identifier, sop, final_data_root)) {
    if (aesthetic_json) {final_data_root.get().merge_patch(*aesthetic_json);}
    final_data_root = final_data_root.get()["data"][0];
  } else {
    // If (optional) aesthetic attributes are available, move final_data_root down one level.
    if (aesthetic_json) {
      final_data_root.get().merge_patch(*aesthetic_json);
      final_data_root = final_data_root.get()["data"][0];
    }
  }

  // If renderable does not contain valid primitive, return constructed sop.
  // Else, return std::nullopt.
  auto maybe_primitive = ProcessPrimitive(frame_name, renderable_name, renderable_json);
  if (maybe_primitive) {
    final_data_root.get().merge_patch(*maybe_primitive);
    return sop;
  } else {
    return std::nullopt;
  }
}

// Process each renderable. To return Json, the input_json must include at least one valid
// "renderable". Else, std::nullopt will be returned.
std::optional<Json> ProcessRenderables(const Json& frame_json, const std::string& frame_name) {
  auto renderables = serialization::TryGetFromMap<Json>(frame_json, "renderables");

  // If no "renderables" key included, return std::nullopt
  if (!renderables) {
    LOG_WARNING("Could not find key \"renderables\" in frame \"%s\". Frame \"%s\" will not be "
                "included in conversion.", frame_name.c_str(), frame_name.c_str());
    return std::nullopt;
  } else {
    Json frame_data;

    // Iterate over "renderables" and add data if valid renderable is generated
    for (const auto &renderable_it : (*renderables).items()) {
      auto renderable_data =
        ProcessRenderable(renderable_it.value(), renderable_it.key(), frame_name);
        if (renderable_data) { frame_data.push_back(*renderable_data); }
    }

    // If at least one valid "renderable" was found, return frame_data. Else, return std::nullopt
    if (!frame_data.empty()) {
      return frame_data;
    } else {
      LOG_WARNING("Could not find any valid \"renderables\" in frame \"%s\". Frame \"%s\" will not "
                  "be included in conversion.", frame_name.c_str(), frame_name.c_str());
      return std::nullopt;
    }
  }
}

// Process each frame. To return Json, the frame must include (1) "frame_id" and/or "pose", and (2)
// at least one valid renderable. Else, std::nullopt will be returned.
std::optional<Json> ProcessFrame(const Json& frame_json, const std::string& frame_name) {
  Json frame_sop;
  auto final_data_root = std::ref(frame_sop);

  // If transform not available return std::nullopt, else look for aesthetic attributes and
  // renderables.
  const std::string element_identifier = "frame \"" + frame_name + "\"";
  if (!ProcessTransform(frame_json, element_identifier, frame_sop, final_data_root)) {
    LOG_WARNING("Could not find a valid \"frame_id\" or \"pose\" for frame \"%s\". Frame \"%s\" "
                "will not be included in conversion.", frame_name.c_str(), frame_name.c_str());
    return std::nullopt;
  } else {
    // Add (optional) aesthetic attributes if available
    auto aesthetic_json = ProcessAesthetics(frame_json, element_identifier);
    if (aesthetic_json) {final_data_root.get().merge_patch(*aesthetic_json);}

    auto frame_data = ProcessRenderables(frame_json, frame_name);

    // If valid renderables were found return processed data, else return std::nullopt.
    if (frame_data) {
      final_data_root.get()["data"] = *frame_data;
      return frame_sop;
    } else {
      return std::nullopt;
    }
  }
}

}  // namespace kinematic_json_details

Json FromKinematicJson(const Json& json) {
  // Initialize empty named Sop with basic structure and name:
  Json named_sop;
  named_sop["automatic_kinematic_json_conversion"]["type"] = "sop";
  Json& data = named_sop["automatic_kinematic_json_conversion"]["data"];

  // Iterate over frames, adding data to named_sop:
  for (const auto& frame_it : json.items()) {
    auto frame_data = kinematic_json_details::ProcessFrame(frame_it.value(), frame_it.key());
    if (frame_data) { data.push_back(*frame_data); }
  }

  return named_sop;
}

Json FromKinematicJsonFile(const std::string& filename) {
  return FromKinematicJson(serialization::LoadJsonFromFile(filename));
}

}  // namespace kinematics
}  // namespace isaac
