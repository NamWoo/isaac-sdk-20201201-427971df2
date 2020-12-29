/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <set>
#include <string>
#include <utility>

#include "engine/alice/component.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace alice {

class ConfigBackend;

// Stores node configuration in form of key-value pairs.
//
// This component is added to every node by default and does not have to be added manually.
//
// The config component is used by other components and the node itself to store structure and
// state. Most notable config can be used directly in codelets to access custom configuration
// values. Support for basic types and some math types is built-in.
// Configuration is stored in a group-key-value format. Each component and the node itself are
// defining separate groups of key-value pairs. Additionally custom groups of configuration can be
// added by the user.
class Config : public Component {
 public:
  virtual ~Config() = default;

  void initialize() override;
  void deinitialize() override;

  // Tries to get the configuration value for the given key in a component in this node
  template<typename T>
  std::optional<T> tryGet(Component* component, const std::string& key) const {
    const nlohmann::json* ptr = tryGetJsonFromCentral(component, key);
    if (ptr == nullptr) {
      return std::nullopt;
    }
    return serialization::TryGet<T>(*ptr);
  }

  // Sets the configuration value for the given key in a component in this node. The change might
  // not be visible immediately to a codelet when it is ticking.
  template<typename T>
  void async_set(Component* component, const std::string& key, T value) {
    nlohmann::json json;
    serialization::Set(json, std::move(value));
    writeJsonToCentral(component, key, std::move(json));
  }

  // Gets all configuration for a given component in this node
  nlohmann::json getAll(Component* component) const;

 private:
  friend class CodeletCanister;
  friend class ConfigBackend;
  friend class ConfigHook;
  friend class NodeCanister;

  // Tries to get a JSON value for the given key in a component in this node from central storage
  const nlohmann::json* tryGetJsonFromCentral(const Component* component,
                                              const std::string& key) const;
  // Writes a JSON value for the given key in a component in this node to central storage
  void writeJsonToCentral(const Component* component, const std::string& key,
                          nlohmann::json&& json);
  // Finds a config hook based on component and key
  ConfigHook* findHook(const Component* component, const std::string& key) const;
  // Central storage changes a configuration parameter stored in this Config and notifies this
  // component about it.
  void updateHookFromCentral(const Component* component, const std::string& key,
                             const nlohmann::json& json);
  // Updates configuration parameter of a hook by looking up the correct value in central storage.
  void setHookFromCentral(ConfigHook* hook);
  // Gets the key used to identify a component in this node in the configuration database
  std::string getComponentKey(const Component* component) const;
  // Adds a config hook
  void addConfigHook(ConfigHook* hook);
  // Marks a hook as dirty
  void markConfigHookAsDirty(ConfigHook* hook);
  // Updates all hooks for a components. Also marks dirty hooks as clean.
  void updateHooks(Component* component);
  // Update the central config with the latest value of all the hooks of a given component.
  void updateCentralFromHooks(Component* component);
  // Updates all hooks for a component which were marked dirty and sets their state to clean.
  void updateDirtyHooks(Component* component);

  ConfigBackend* backend_;

  mutable std::mutex hooks_mutex_;
  std::set<ConfigHook*> hooks_;
  std::set<ConfigHook*> dirty_hooks_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::Config)
