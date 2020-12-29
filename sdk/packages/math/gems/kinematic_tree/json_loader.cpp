/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "json_loader.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "engine/core/math/pose3.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/serialization/json_formatter.hpp"
#include "packages/math/gems/kinematic_tree/constant_motor.hpp"
#include "packages/math/gems/kinematic_tree/kinematic_tree.hpp"
#include "packages/math/gems/kinematic_tree/motor.hpp"
#include "packages/math/gems/kinematic_tree/revolute_motor.hpp"

namespace isaac {
namespace kinematic_tree {

namespace {

// Helper constexpr
constexpr char kPropertiesKey[] = "properties";
constexpr char kPoseKey[] = "pose";
constexpr char kAxisKey[] = "axis";
constexpr char kLimitsKey[] = "limits";
constexpr char kParentKey[] = "parent";
constexpr char kMotorKey[] = "motor";
constexpr char kTypeKey[] = "type";
constexpr char kLinksKey[] = "links";
constexpr char kNameKey[] = "name";

// Loads a Motor from a json
Motor* GetMotorFromJson(const Json& json) {
  if (json.find(kTypeKey) == json.end()) {
    return nullptr;
  }
  switch (json[kTypeKey].get<MotorType>()) {
    case MotorType::kConstantMotor: {
      auto properties = json.find(kPropertiesKey);
      if (properties == json.end()) {
        LOG_ERROR("Missing `%s` for ConstantMotor", kPropertiesKey);
        return nullptr;
      }
      const auto maybe_pose = serialization::TryGetFromMap<Pose3d>(*properties, kPoseKey);
      if (!maybe_pose) {
        LOG_ERROR("Missing or invalid `%s` for ConstantMotor", kPoseKey);
        return nullptr;
      }
      return new ConstantMotor(*maybe_pose);
    }
    case MotorType::kRevoluteMotor: {
      auto properties = json.find(kPropertiesKey);
      if (properties == json.end()) {
        LOG_ERROR("Missing `%s` for RevoluteMotor", kPropertiesKey);
        return nullptr;
      }
      const auto maybe_axis = serialization::TryGetFromMap<Vector3d>(*properties, kAxisKey);
      if (!maybe_axis) {
        LOG_ERROR("Missing or invalid `%s` for RevoluteMotor", kAxisKey);
        return nullptr;
      }
      const auto maybe_limits = serialization::TryGetFromMap<Vector2d>(*properties, kLimitsKey);
      if (maybe_limits) {
        return new RevoluteMotor(*maybe_axis, *maybe_limits);
      } else {
        return new RevoluteMotor(*maybe_axis);
      }
    }
    default:
      LOG_ERROR("Motor type `%s` unknown", json[kTypeKey].dump(2).c_str());
      return nullptr;
  }
}

}  //  namespace

bool FromJson(const Json& json, KinematicTree& model) {
  const auto links = json.find(kLinksKey);
  if (links == json.end()) return false;
  // iterate through json and add name of link to sorted list if its parent is in that list
  std::vector<std::string> order;
  std::unordered_map<std::string, int> order_mapping;
  std::unordered_map<std::string, std::string> remains;
  std::unordered_set<std::string> processed;
  for (const auto& value : *links) {
    const auto name = value.find(kNameKey);
    if (name == value.end()) {
      LOG_ERROR("Failed to load the link, the `%s` is missing", kNameKey);
      return false;
    }
    const std::string name_str = name->get<std::string>();
    order_mapping[name_str] = order_mapping.size();
    const auto parent = value.find(kParentKey);
    if (parent == value.end()) {
      order.push_back(name_str);
      processed.insert(name_str);
    } else {
      const std::string parent_str = parent->get<std::string>();
      if (processed.count(parent_str) == 1) {
        order.push_back(name_str);
        processed.insert(name_str);
      } else {
        remains.insert({name_str, parent_str});
      }
    }
  }
  // Try to find a valid order (A parent need to be added before its children).
  while (!remains.empty()) {
    std::vector<std::string> recently_added;
    for (auto item = remains.begin(); item != remains.end(); item++) {
      if (processed.count(item->first) == 1) {
        order.push_back(item->first);
        processed.insert(item->first);
        recently_added.push_back(item->first);
      }
    }
    if (recently_added.empty()) {
      for (const auto& it : remains) {
        LOG_ERROR("Link `%s` does not have its parent `%s`", it.first.c_str(), it.second.c_str());
      }
      return false;
    }
    for (const std::string& link_name : recently_added) {
      remains.erase(link_name);
    }
  }
  // add links to try in order. parent must be added before child
  for (const auto& name : order) {
    const Json& value = (*links)[order_mapping[name]];
    // get motor. default to identity transform if not provided
    std::unique_ptr<Motor> motor;
    if (value.find(kMotorKey) != value.end()) {
      motor.reset(GetMotorFromJson(value[kMotorKey]));
      if (!motor) {
        return false;
      }
    }
    // get parent
    if (value.find(kParentKey) != value.end()) {
      std::string parent = value[kParentKey].get<std::string>();
      model.addLink(name, parent, std::move(motor));
    } else {
      model.addLink(name, /* parent = */ nullptr, std::move(motor));
    }
  }
  return true;
}

}  // namespace kinematic_tree
}  // namespace isaac
