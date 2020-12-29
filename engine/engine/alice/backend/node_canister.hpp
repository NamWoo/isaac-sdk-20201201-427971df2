/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "engine/alice/backend/component_query.hpp"
#include "engine/alice/backend/lifecycle.hpp"
#include "engine/gems/uuid/uuid.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace alice {

class Application;
class Codelet;
class CodeletCanister;
class Component;
class Node;

// A helper class which handles the life cylce of a node.
//
// Every node is wrapped into a canister after it is constructed. The canister takes care of the
// life cycle of the node. In particular it provides an API to the node which should only be
// accessible from withing the Isaac SDK Engine and not to codelet developers.
class NodeCanister {
 public:
  // Creates a new canister with a new node inside. The node is setup with basic components.
  NodeCanister(Application* app, std::string name);

  // Destroys the canister and the node it contains. All components of the node are destroyed with
  // it.
  ~NodeCanister();

  // Pointer to the application which contains this node.
  Application* app() const { return app_; }

  // Returns a pointer to the node inside the canister.
  Node* node() const { return node_; }

  // Returns the current lifecycle stage of the node
  LifecycleStage getStage() const { return stage_; }

  // Creates a new component of given type and name.
  Component* addComponent(const std::string& type_name, std::string name);

  // Gets the status of the node based on the status of its components
  Status getStatus() const;

  // Starts the node and all its components
  void start();

  // Starts the codelet now. This is used by NodeBackend when a thread is scheduled to start
  // a codelet due to a call to `requestCodeletStart`.
  void startCodeletImmediately(Codelet* codelet) const;

  // Stops the node and all its components
  void stop();

  // Deinitializes the nodes and all its components
  void deinitialize();

  // Gets statistics about the node as JSON
  nlohmann::json getNodeStatistics() const;
  // Gets statistics about all codelets in the node as JSON
  nlohmann::json getCodeletStatistics() const;

  // Sets the status for all codelets contained within this node canister to the given `status`.
  void setStatusForCodelets(Status status);

 private:
  // Statistics about a node
  struct NodeStatistics {
    // The number of times the node was started
    int num_started;
  };

  // Gets a list with all codelet canisters
  std::vector<CodeletCanister*> getCodeletCanisters() const;

  // Helper function to start all components in the node.
  void startCodelets();
  // Helper function to stop all components in the node.
  void stopCodelets() const;
  // Helper function to deinitialize all components in the node.
  void deinitializeComponents();

  Application* app_;
  Node* node_;

  LifecycleStage stage_;

  // A list which holds ownership of all components
  std::vector<std::unique_ptr<Component>> components_;

  mutable std::mutex codelet_canisters_mutex_;
  std::vector<std::unique_ptr<CodeletCanister>> codelet_canisters_;

  NodeStatistics statistics_;
};

}  // namespace alice
}  // namespace isaac
