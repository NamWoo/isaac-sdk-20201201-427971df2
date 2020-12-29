/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

#include "engine/alice/hooks/hook.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace alice {

class Component;
class Config;

// Common base class for configuration hooks, aka parameters
class ConfigHook : public Hook {
 public:
  // Creates a parameter and associates it with a component
  ConfigHook(Component* component, const char* key);

  // The name of the configuration value
  const std::string& key() const { return key_; }

  // The name of the type of this configuration parameter in a human-readable form
  virtual std::string type_name() const = 0;

  // Sets the parameter of the value by deserializing it from a JSON object. If the deserialization
  // fails false is returned.
  virtual bool set(const nlohmann::json& json) = 0;

  // Gets the default value (or a null object if no default is set)
  virtual nlohmann::json getDefault() const = 0;

  // Updates the central config with the current value hold by this hook. This is used mostly to let
  // the central config know about the potential default value.
  virtual void updateCentralConfig() const = 0;

  // Returns true if this parameter will only be changed when the Config component is instructed
  // to flush its cache. This is used by Codelets to make configuration immutable for the duration
  // of a call to tick/start/stop.
  bool isCached() const;

 protected:
  friend class Component;  // for calling connect

  // Demangles config typename for readability
  static std::string ConfigTypenameDemangle(const std::string& type_name);

  virtual ~ConfigHook() = default;

  // Connects the parameter to the configuration and initializes default values
  void connect() override;

  // Helper function to read the value of this configuration parmaeter from the configuration
  // backend. Returns a nullptr in case the value is not set in the configuration backend.
  const nlohmann::json* readFromBackend() const;

  // Helper function to write the value of this configuration parameter to the configuration
  // backend. In case the value is already set it will be overwritten.
  void writeToBackend(nlohmann::json&& json) const;

  // Marks the hook as dirty
  void markDirty();

 private:
  std::string key_;
  Config* config_;
};

// Parameters are used by components to gives access to values stored in configuration. They are
// automatically registered on object creation and thus can also be used to get component
// configuration.
template <typename T>
class Parameter : public ConfigHook {
 public:
  // Disallow copying for parameters
  Parameter(const Parameter&) = delete;
  Parameter& operator=(const Parameter&) = delete;

  // Creates a parameter and associates it with a component
  Parameter(Component* component, const char* key) : ConfigHook(component, key) {}
  Parameter(Component* component, const char* key, T default_value)
      : ConfigHook(component, key), default_value_(std::move(default_value)) {
    current_value_ = default_value_;
  }

  std::string type_name() const override {
    return ConfigTypenameDemangle(typeid(T).name());
  }

  bool set(const nlohmann::json& json) override {
    auto maybe = serialization::TryGet<T>(json);
    if (maybe) {
      setImpl(std::move(*maybe));
      return true;
    } else {
      return false;
    }
  }

  nlohmann::json getDefault() const override {
    nlohmann::json result;
    if (default_value_) {
      serialization::Set(result, *default_value_);
    }
    return result;
  }

  // Gets the value of the parameter. This function will assert in case the parameter does not
  // exist.
  T get() const {
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT(current_value_ != std::nullopt, "Parameter '%s/%s' not found or wrong type",
           component()->full_name().c_str(), key().c_str());
    return *current_value_;
  }

  // Tries to get the parameter. This function will return nullopt in case the parameter does not
  // exist.
  std::optional<T> tryGet() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return current_value_;
  }

  // Sets the value of the parameter. This will write the new value to the configuration backend.
  // In case `force_hook_update` is set to true or if this is not a cached component the cached
  // value in the hook will also be immediately set to the new value.
  void set(T&& value, bool force_hook_update) {
    // Determine if the value stored in the hook will be updated now.
    const bool immediate = !isCached() || force_hook_update;
    // Warning: This function assumes that a tick can not start happening during the execution of
    // this function when `immediate` is true. This should be true as non-cached hooks are only
    // found on components which do not tick while forced updates should only happen from within the
    // component.
    // Write the new value to the backend.
    nlohmann::json json;
    serialization::Set(json, value);
    writeToBackend(std::move(json));
    // Either set the value or mark the hook as dirty.
    if (immediate) {
      setImpl(std::move(value));
    } else {
      markDirty();
    }
  }

 protected:
  void updateCentralConfig() const override {
    if (current_value_) {
      nlohmann::json json;
      serialization::Set(json, *current_value_);
      writeToBackend(std::move(json));
    }
  }
  // Sets the value which is cached in the hook. This is called by the config backend whenever the
  // configuration changes.
  void setImpl(T&& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    current_value_ = std::move(value);
  }

  // Protecting concurrent calls to get and set
  mutable std::mutex mutex_;

  // The current value of the parameter
  std::optional<T> current_value_;

  // The default value to use in case the parameter was not set in the configuration
  std::optional<T> default_value_;
};

}  // namespace alice
}  // namespace isaac

// This macro can be used to add a configuration parameter to a component and will create the
// corresponding member variable and access functions. The configuration parameter will not be
// set by default. The `try_get_xxx` function can be used to access a configuration value if it
// is not known if a parameter was set.
#define _ISAAC_PARAM_IMPL_1(TYPE, KEY)                                                \
                                                                                      \
 private:                                                                             \
  isaac::alice::Parameter<TYPE> param_##KEY##_{this, #KEY};                           \
                                                                                      \
 protected:                                                                           \
  void set_##KEY(TYPE x) { param_##KEY##_.set(std::move(x), true); }                  \
                                                                                      \
 public:                                                                              \
  void async_set_##KEY(TYPE x) { param_##KEY##_.set(std::move(x), false); }         \
                                                                                      \
 public:                                                                              \
  std::optional<TYPE> try_get_##KEY() const { return param_##KEY##_.tryGet(); }       \
  TYPE get_##KEY() const { return param_##KEY##_.get(); }

// This macro can be used to add a configuration parameter to a component and will create the
// corresponding member variable and access functions. The configuration parameter will be
// initialized from the beginning with the provided default value.
#define _ISAAC_PARAM_IMPL_2(TYPE, KEY, DEFAULT)                                       \
                                                                                      \
 private:                                                                             \
  isaac::alice::Parameter<TYPE> param_##KEY##_{this, #KEY, DEFAULT};                  \
                                                                                      \
 protected:                                                                           \
  void set_##KEY(TYPE x) { param_##KEY##_.set(std::move(x), true); }                  \
                                                                                      \
 public:                                                                              \
  void async_set_##KEY(TYPE x) { param_##KEY##_.set(std::move(x), false); }         \
                                                                                      \
 public:                                                                              \
  TYPE get_##KEY() const { return param_##KEY##_.get(); }

// The sliding macro trick is used to enable macro overloading.
// This additional EVALUATE macro is required to be compatible with MSVC
#define _ISAAC_PARAM_EVALUATE(x) x
#define _ISAAC_PARAM_GET_OVERRIDE(_1, _2, _3, NAME, ...) NAME
#define ISAAC_PARAM(...)                                                            \
  _ISAAC_PARAM_EVALUATE(_ISAAC_PARAM_GET_OVERRIDE(__VA_ARGS__, _ISAAC_PARAM_IMPL_2, \
                                                  _ISAAC_PARAM_IMPL_1)(__VA_ARGS__))
