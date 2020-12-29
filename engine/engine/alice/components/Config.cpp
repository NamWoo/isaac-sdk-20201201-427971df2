/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Config.hpp"

#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

void Config::initialize() {
  backend_ = node()->app()->backend()->getBackend<ConfigBackend>();
}

void Config::deinitialize() {
  backend_ = nullptr;
}

nlohmann::json Config::getAll(Component* component) const {
  return backend_->getAllForComponent(node()->name(), getComponentKey(component));
}

const nlohmann::json* Config::tryGetJsonFromCentral(const Component* component,
                                                    const std::string& key) const {
  return backend_->tryGetJson(node()->name(), getComponentKey(component), key);
}

void Config::writeJsonToCentral(const Component* component, const std::string& key,
                                nlohmann::json&& json) {
  if (ConfigHook* hook = findHook(component, key)) {
    std::unique_lock<std::mutex> lock(hooks_mutex_);
    dirty_hooks_.insert(hook);
  }
  // When a parameter is changed on the request of the config component the backend does not need
  // to notify that component.
  const bool kNotifyConfigComponent = false;
  backend_->setJsonImpl(node()->name(), getComponentKey(component), key, std::move(json),
                        kNotifyConfigComponent);
}

ConfigHook* Config::findHook(const Component* component, const std::string& key) const {
  std::unique_lock<std::mutex> lock(hooks_mutex_);
  const auto it = std::find_if(hooks_.begin(), hooks_.end(),
    [&] (const ConfigHook* hook) {
      return hook->component() == component && hook->key() == key;
    });
  return (it == hooks_.end()) ? nullptr : *it;
}

void Config::updateHookFromCentral(const Component* component, const std::string& key,
                                   const nlohmann::json& json) {
  ASSERT(component != nullptr, "argument must not be null");
  ConfigHook* hook = findHook(component, key);
  if (hook == nullptr) {
    LOG_WARNING("Could not find hook: %s/%s/%s", component->node()->name().c_str(),
              component->name().c_str(), key.c_str());
    return;
  }
  if (hook->isCached()) {
    markConfigHookAsDirty(hook);
  } else {
    if (!hook->set(json)) {
      LOG_ERROR("Could not deserialize configuration parameter to necessary type");
    }
  }
}

void Config::setHookFromCentral(ConfigHook* hook) {
  const auto maybe_json = backend_->tryGetJson(hook->component()->node()->name(),
                                               hook->component()->name(), hook->key());
  if (maybe_json) {
    if (!hook->set(*maybe_json)) {
      LOG_ERROR("Could not deserialize configuration parameter '%s/%s/%s to necessary type",
                hook->component()->node()->name().c_str(), hook->component()->name().c_str(),
                hook->key().c_str());
    }
  }
}

std::string Config::getComponentKey(const Component* component) const {
  if (component == nullptr) {
    return "_";
  } else {
    ASSERT(component->node() == this->node(), "Component not part of this node");
    return component->name();
  }
}

void Config::addConfigHook(ConfigHook* hook) {
  std::unique_lock<std::mutex> lock(hooks_mutex_);
  hooks_.insert(hook);
}

void Config::markConfigHookAsDirty(ConfigHook* hook) {
  ASSERT(&hook->component()->node()->config() == this,
         "This hook is not managed by this config component");
  std::unique_lock<std::mutex> lock(hooks_mutex_);
  dirty_hooks_.insert(hook);
}

void Config::updateCentralFromHooks(Component* component) {
  std::vector<ConfigHook*> hooks;
  {
    std::unique_lock<std::mutex> lock(hooks_mutex_);
    for (auto it = hooks_.begin(); it != hooks_.end(); ++it) {
      ConfigHook* hook = *it;
      if (hook->component() == component) {
        hooks.push_back(hook);
      }
    }
  }
  for (const ConfigHook* hook : hooks) {
    hook->updateCentralConfig();
  }
}

void Config::updateHooks(Component* component) {
  std::unique_lock<std::mutex> lock(hooks_mutex_);
  for (auto it = hooks_.begin(); it != hooks_.end(); ++it) {
    ConfigHook* hook = *it;
    if (hook->component() == component) {
      setHookFromCentral(hook);
      dirty_hooks_.erase(hook);
    }
  }
}

void Config::updateDirtyHooks(Component* component) {
  std::unique_lock<std::mutex> lock(hooks_mutex_);
  // The following loop erases while iteration by using a standard conforming pattern.
  for (auto it = dirty_hooks_.begin(); it != dirty_hooks_.end(); ) {
    ConfigHook* hook = *it;
    if (hook->component() == component) {
      setHookFromCentral(hook);
      dirty_hooks_.erase(it++);
    } else {
      ++it;
    }
  }
}

}  // namespace alice
}  // namespace isaac
