/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "config_backend.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

void ConfigBackend::set(const nlohmann::json& json) {
  for (auto it1 = json.begin(); it1 != json.end(); ++it1) {
    const std::string& node_name = it1.key();
    for (auto it2 = it1->begin(); it2 != it1->end(); ++it2) {
      const std::string& component_name = it2.key();
      if (it2->is_object()) {
        // This is a json object, thus it's config for a component
        for (auto it3 = it2->begin(); it3 != it2->end(); ++it3) {
          const std::string& parameter_name = it3.key();
          nlohmann::json copy = *it3;
          setJson(node_name, component_name, parameter_name, std::move(copy));
        }
      } else {
        // This is a base value, thus it's config for the node
        nlohmann::json copy = *it2;
        setJson(node_name, component_name, "", std::move(copy));
      }
    }
  }
}

bool ConfigBackend::erase(const std::string& node_name, const std::string& component_name,
                          const std::string& parameter_name) {
  std::unique_lock<std::shared_timed_mutex> lock(cache_mutex_);
  const auto it = cache_.find(GetCacheKey(node_name, component_name, parameter_name));
  if (it != cache_.end()) {
    cache_.erase(it);
    return true;
  }
  return false;
}

const nlohmann::json* ConfigBackend::tryGetJson(const std::string& node_name,
                                                const std::string& component_name,
                                                const std::string& parameter_name) const {
  std::shared_lock<std::shared_timed_mutex> lock(cache_mutex_);
  const auto it = cache_.find(GetCacheKey(node_name, component_name, parameter_name));
  return (it == cache_.end()) ? nullptr : &it->second;
}

void ConfigBackend::setJson(const std::string& node_name, const std::string& component_name,
                            const std::string& parameter_name, nlohmann::json&& json) {
  // When parameters are changed via backend the config component needs to be optimized.
  constexpr bool kNnotifyConfigComponent = true;
  setJsonImpl(node_name, component_name, parameter_name, std::move(json), kNnotifyConfigComponent);
}

nlohmann::json ConfigBackend::getAllForComponent(const std::string& node_name,
                                                 const std::string& component_name) const {
  std::unique_lock<std::shared_timed_mutex> lock(json_root_mutex_);
  auto it1 = json_root_.find(node_name);
  if (it1 == json_root_.end()) {
    return {};
  }
  auto it2 = it1->find(component_name);
  if (it2 == it1->end()) {
    return {};
  }
  return *it2;
}

std::string ConfigBackend::GetCacheKey(const std::string& node, const std::string& component,
                                       const std::string& key) {
  return node + "/" + component + "/" + key;
}

void ConfigBackend::setJsonImpl(const std::string& node_name, const std::string& component_name,
                                const std::string& parameter_name, nlohmann::json&& json,
                                bool notify_config_component) {
  std::unique_lock<std::shared_timed_mutex> lock(json_root_mutex_);
  auto it_node = json_root_.find(node_name);
  if (it_node == json_root_.end()) {
    json_root_[node_name] = {};
    it_node = json_root_.find(node_name);
  }
  auto it_component = it_node->find(component_name);
  if (it_component == it_node->end()) {
    (*it_node)[component_name] = {};
    it_component = it_node->find(component_name);
  }
  if (!parameter_name.empty()) {
    // set a config for a component
    auto it_parameter = it_component->find(parameter_name);
    if (it_parameter == it_component->end()) {
      (*it_component)[parameter_name] = nlohmann::json{};
      it_parameter = it_component->find(parameter_name);
    }
    *it_parameter = std::move(json);
    // Also add the value to the cache
    {
      std::unique_lock<std::shared_timed_mutex> lock(cache_mutex_);
      cache_[GetCacheKey(node_name, component_name, parameter_name)] = *it_parameter;
    }
    // Notify the Config component about the change
    if (notify_config_component) {
      // TODO(dweikersdorf) This is quite slow, however caching nodes/components is difficult as the
      //     might get deleted over the runtime of the app.
      Node* node = app()->findNodeByName(node_name);
      if (node != nullptr) {
        Component* component = node->findComponentByName(component_name);
        if (component != nullptr) {
          node->config().updateHookFromCentral(component, parameter_name, *it_parameter);
        }
      }
    }
  } else {
    // set a config for the node
    *it_component = std::move(json);
  }
}

}  // namespace alice
}  // namespace isaac
