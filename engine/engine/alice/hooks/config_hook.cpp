/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "config_hook.hpp"

#include <string>
#include <unordered_map>
#include <utility>

#include "engine/alice/component.hpp"
#include "engine/alice/components/Codelet.hpp"
#include "engine/alice/components/Config.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

std::string ConfigHook::ConfigTypenameDemangle(const std::string& type_name) {
  static std::unordered_map<std::string, std::string> kLookup {
    {"b", "bool"},
    {"i", "int"},
    {"d", "double"},
    {"NSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE", "string"},
    {"St6vectorINSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESaIS5_EE", "vector<string>"},
    {"N8nlohmann10basic_jsonISt3mapSt6vectorNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEblmdSaNS_14adl_serializerEEE", "json"},  // NOLINT
    {"N5isaac5Pose3IdEE", "Pose3"},
    {"N5Eigen6MatrixIdLi2ELi1ELi0ELi2ELi1EEE", "Vector2d"},
    {"N5Eigen6MatrixIiLi2ELi1ELi0ELi2ELi1EEE", "Vector2i"},
    {"N5Eigen6MatrixIdLi3ELi1ELi0ELi3ELi1EEE", "Vector3d"},
    {"N5Eigen6MatrixIiLi3ELi1ELi0ELi3ELi1EEE", "Vector3i"},
    {"N5Eigen6MatrixIdLi4ELi1ELi0ELi6ELi1EEE", "Vector4d"},
    {"N5Eigen6MatrixIdLi5ELi1ELi0ELi6ELi1EEE", "Vector5d"},
    {"N5Eigen6MatrixIdLi6ELi1ELi0ELi6ELi1EEE", "Vector6d"}
  };
  const auto it = kLookup.find(type_name);
  if (it == kLookup.end()) {
    return type_name;
  } else {
    return it->second;
  }
}

ConfigHook::ConfigHook(Component* component, const char* key)
: Hook(component), key_(key) {}

void ConfigHook::connect() {
  config_ = component()->node()->getComponent<Config>();
  config_->addConfigHook(this);
}

bool ConfigHook::isCached() const {
  return dynamic_cast<Codelet*>(component()) != nullptr;
}

const nlohmann::json* ConfigHook::readFromBackend() const {
  return config_->tryGetJsonFromCentral(component(), key_);
}

void ConfigHook::writeToBackend(nlohmann::json&& json) const {
  // Write the new value to the backend. We don't want the Config component to be notified about
  // the update as we are aware already.
  config_->writeJsonToCentral(component(), key_, std::move(json));
}

void ConfigHook::markDirty() {
  config_->markConfigHookAsDirty(this);
}

}  // namespace alice
}  // namespace isaac
