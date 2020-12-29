/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/backend/component_query.hpp"
#include "engine/alice/backend/lifecycle.hpp"
#include "engine/alice/component.hpp"
#include "engine/alice/components/Config.hpp"
#include "engine/alice/components/Pose.hpp"
#include "engine/alice/components/Sight.hpp"

namespace isaac {
namespace alice {

class Application;
class Clock;
class NodeCanister;
class PybindNode;

// An application consists of many nodes which each fulfills a certain purpose. For example
// localizing the robot or planning a path could be implemented via a node. Nodes are created out
// of various components which define the node's functionality.
class Node {
 public:
  // The name of this node is useful for human readability but must not be unique
  const std::string& name() const { return name_; }

  // The app for this node
  Application* app();

  // The clock backend for this node
  Clock* clock();

  // Returns the current lifecycle stage of the node
  LifecycleStage getStage() const;

  // Creates a new component with given type name and name
  Component* addComponent(const std::string& type_name, std::string name);
  // Adds new component with given name and type to the node. Components must have unique
  // names within a node, otherwise the function will assert.
  template <typename T>
  T* addComponent(std::string name);
  // Convient function similar to `addComponent` which uses the component type name as name. Beware
  // that you can not call this function more than once as the component name must be unique.
  template <typename T>
  T* addComponent();

  // Checks if this node has a component of the given type
  template <typename T>
  bool hasComponent() const;
  // Returns true if this node has a component with this name
  bool hasComponentByName(const std::string& name) const;

  // Gets the unique component with the given type. Asserts if there are multiple or no components
  // of that type.
  template <typename T>
  T* getComponent() const;
  // Gets component with the given type, or null if no such component or multiple components
  template <typename T>
  T* getComponentOrNull() const;
  // Gets all components with the given type
  template <typename T>
  std::vector<T*> getComponents() const;
  // Gets all components
  std::vector<Component*> getComponents() const;
  // Gets component with the given name, or null if no such component exists
  Component* findComponentByName(const std::string& name) const;
  // Gets the component for the given type and if none exists creates one first
  template <typename T>
  T* getOrAddComponent();

  // Gets the config component of this node (this component always exists)
  Config& config() const { return getComponentCached(config_component_); }
  // Gets the pose component (this component always exists)
  Pose& pose() const { return getComponentCached(pose_component_); }
  // Gets the sight visualization component (this component always exists)
  Sight& sight() const { return getComponentCached(sight_component_); }

  // Gets the status of the node based on the status of its components
  Status getStatus() const;

  // Set this to true if the node should not be started automatically when the applications starts.
  // Note that nodes created at runtime are never started automatically and always need to be
  // started manually with app->backend()->node_backed()->start(my_node);
  bool disable_automatic_start = false;

  // The order in which nodes will be started. The smaller the earliers. Note that this of course
  // only applies to nodes which are started at the same time. If a node is started dynamically this
  // does not have any effect.
  int start_order = 0;

 private:
  friend class NodeCanister;
  friend class PybindNode;

  // Derives a name for a component from its type
  static std::string ComputeNameFromType(const std::string& type);

  // Used by NodeCanister to create new nodes
  Node(std::string name);
  // Used by NodeCanister to add components to the list of components
  void addComponent(Component* component);

  // Implementation of `hasComponent<T>`
  bool hasComponent(const ComponentQuery& query) const;
  // Implementation of `getComponent<T>`
  Component* getComponent(const ComponentQuery& query) const;
  // Implementation of `getComponentOrNull<T>`
  Component* getComponentOrNull(const ComponentQuery& query) const;
  // Implementation of `getComponents<T>`
  std::vector<Component*> getComponents(const ComponentQuery& query) const;

  // Gets a component via a cache. This will get the component only once and from there one use
  // the cache instead.
  template <typename Component>
  Component& getComponentCached(Component*& component) const {
    if (component == nullptr) {
      component = getComponent<Component>();
    }
    return *component;
  }

  std::string name_;

  NodeCanister* canister_;

  // A list of components in this node. The components are owned by the node canister.
  std::vector<Component*> components_;
  // A mutex protecting the member variable `component_` from concurrent access.
  mutable std::mutex components_mutex_;

  mutable Config* config_component_;
  mutable Pose* pose_component_;
  mutable Sight* sight_component_;
};

// Implementation

template <typename T>
T* Node::addComponent() {
  return addComponent<T>(ComputeNameFromType(ComponentName<T>::TypeName()));
}

template <typename T>
T* Node::addComponent(std::string name) {
  return static_cast<T*>(addComponent(ComponentName<T>::TypeName(), std::move(name)));
}

template <typename T>
bool Node::hasComponent() const {
  return hasComponent(CreateComponentTypeQuery<T>());
}

template <typename T>
T* Node::getComponent() const {
  return static_cast<T*>(getComponent(CreateComponentTypeQuery<T>()));
}

template <typename T>
T* Node::getComponentOrNull() const {
  return static_cast<T*>(getComponentOrNull(CreateComponentTypeQuery<T>()));
}

template <typename T>
std::vector<T*> Node::getComponents() const {
  const std::vector<Component*> components = getComponents(CreateComponentTypeQuery<T>());
  std::vector<T*> result(components.size());
  std::transform(components.begin(), components.end(), result.begin(),
                 [](Component* component) { return static_cast<T*>(component); });
  return result;
}

template <typename T>
T* Node::getOrAddComponent() {
  if (hasComponent<T>()) {
    return getComponent<T>();
  } else {
    return addComponent<T>();
  }
}

}  // namespace alice
}  // namespace isaac
