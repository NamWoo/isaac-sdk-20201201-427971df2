/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/status.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace alice {

class Hook;
class Node;
class NodeBackend;
class Pose;

// Components are the essential basic building blocks of every node
class Component {
 public:
  virtual ~Component();

  // The node which this component is part of
  Node* node() const { return node_; }

  // This function is called during the initialization phase of the component. It allows derived
  // classes to execute custom code during the initialization phase.
  virtual void initialize() {}

  // This function is called during the deinitialization phase of the component. It allows derived
  // classes to execute custom code during the deinitialization phase. `deinitialize` is guaranteed
  // to be called when and after `initialize` was called.
  virtual void deinitialize() {}

  // The name of the base type of this component, e.g. Codelet
  const std::string& base_type_name() const { return base_name_; }
  // The name of the type of this component, e.g. Config
  const std::string& type_name() const { return type_name_; }
  // The name of the component
  const std::string& name() const { return name_; }
  // This value can be used to uniquely identify each component across all applications
  const Uuid& uuid() const { return uuid_; }
  // The full name of the component including the node
  std::string full_name() const;

  // Adds a hook to the component
  void addHook(Hook* hook) { hooks_.push_back(hook); }
  // Removes the hook from the component
  void removeHook(Hook* hook);
  // Internal function called after the component is added to the node
  void connectHooks();

  // Returns a list with all hooks associated with this component
  const std::vector<Hook*>& hooks() const { return hooks_; }

 private:
  friend class NodeCanister;
  friend class ComponentRegistry;

  std::string base_name_;
  std::string type_name_;
  std::string name_;
  Uuid uuid_;
  Node* node_ = nullptr;

  // To connect hooks
  std::vector<Hook*> hooks_;
};

}  // namespace alice
}  // namespace isaac
