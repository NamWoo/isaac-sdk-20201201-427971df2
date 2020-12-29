/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "node.hpp"

#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/node_canister.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

Application* Node::app() {
  ASSERT(canister_ != nullptr, "Canister not set");
  Application* result = canister_->app();
  ASSERT(result, "argument null");
  return result;
}

Clock* Node::clock() {
  Clock* result = app()->backend()->clock();
  ASSERT(result, "argument null");
  return result;
}

LifecycleStage Node::getStage() const {
  ASSERT(canister_ != nullptr, "Canister not set");
  return canister_->getStage();
}

Component* Node::addComponent(const std::string& type_name, std::string name) {
  ASSERT(canister_ != nullptr, "Canister not set");
  return canister_->addComponent(type_name, std::move(name));
}

bool Node::hasComponentByName(const std::string& name) const {
  return findComponentByName(name) != nullptr;
}

Component* Node::findComponentByName(const std::string& name) const {
  std::unique_lock<std::mutex> lock(components_mutex_);
  for (Component* component : components_) {
    if (component->name() == name) {
      return component;
    }
  }
  return nullptr;
}

std::vector<Component*> Node::getComponents() const {
  std::unique_lock<std::mutex> lock(components_mutex_);
  return components_;
}

Status Node::getStatus() const {
  return canister_->getStatus();
}

std::string Node::ComputeNameFromType(const std::string& type) {
  // Replace "::"" from fully qualified C++ typenames with '.' to create a readable name
  return std::regex_replace(type, std::regex("::"), ".");
}

Node::Node(std::string name)
    : name_(std::move(name)), canister_(nullptr), config_component_(nullptr),
      pose_component_(nullptr), sight_component_(nullptr) {
}

void Node::addComponent(Component* component) {
  std::unique_lock<std::mutex> lock(components_mutex_);
  components_.push_back(component);
}

bool Node::hasComponent(const ComponentQuery& query) const {
  std::unique_lock<std::mutex> lock(components_mutex_);
  for (Component* component : components_) {
    if (query.test(component)) {
      return true;
    }
  }
  return false;
}

Component* Node::getComponent(const ComponentQuery& query) const {
  std::unique_lock<std::mutex> lock(components_mutex_);
  Component* result = nullptr;
  for (Component* component : components_) {
    if (query.test(component)) {
      ASSERT(result == nullptr, "Found multiple components of type '%s' in node '%s'. "
             "Use `getComponents` to get multiple components.", query.message.c_str(),
             name().c_str());
      result = component;
    }
  }
  ASSERT(result != nullptr, "Could not find a component of type '%s' in node '%s'. "
         "Use `hasComponent` or `getComponentOrNull` if you are not sure if a component exists.",
         query.message.c_str(), name().c_str());
  return result;
}

Component* Node::getComponentOrNull(const ComponentQuery& query) const {
  std::unique_lock<std::mutex> lock(components_mutex_);
  Component* result = nullptr;
  for (Component* component : components_) {
    if (query.test(component)) {
      if (result != nullptr) {
        return nullptr;
      }
      result = component;
    }
  }
  return result;
}

std::vector<Component*> Node::getComponents(const ComponentQuery& query) const {
  std::unique_lock<std::mutex> lock(components_mutex_);
  std::vector<Component*> result;
  for (Component* component : components_) {
    if (query.test(component)) {
      result.push_back(component);
    }
  }
  return result;
}

}  // namespace alice
}  // namespace isaac
