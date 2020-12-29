/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "engine/alice/node.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/scheduler/job_descriptor.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

class Application;
class Component;
class Node;
class NodeCanister;
class Prefix;

// Handles node and component lifetime
class NodeBackend {
 public:
  // A status update used by NodeCanister and ComponentCanister
  struct StatusUpdate {
    // The timestamp at which the status update happened
    int64_t timestamp;
    // The name of the node which changed status
    std::string node_name;
    // The name of the component which changed status
    std::string component_name;
    // The old life cycle before the status change
    LifecycleStage old_stage;
    // The new life cycle after the status change
    LifecycleStage new_stage;
  };

  NodeBackend(Application* app);
  ~NodeBackend();

  // The app to which this node backend belongs
  Application* app() const { return app_; }

  // Creates a new node. `name` is user-defined unique name for the node. This function will assert
  // if the name is not unique.
  Node* createNode(const std::string& name);
  // Like `createNode` and also adds message passing capabilities
  Node* createMessageNode(const std::string& name);

  // Destroys a node and all its components.
  void destroyNode(const std::string& name);

  // Finds a node by name. Will return nullptr if no node with this name exists.
  Node* findNodeByName(const std::string& name) const;

  // Returns the number of nodes
  size_t numNodes() const;

  // Returns a list of all nodes
  std::vector<Node*> nodes() const;

  // Loads nodes from JSON
  // The top-level structure is an array of nodes where each node is a dictionary. Each node has
  // certain fields and among them is one called 'components'. This is an array itself where
  // each component again is a dictionary with certain fields.
  // An example:
  //   {
  //     "name": "myname",
  //     "components": [
  //       {"name": "comp1", "type": "type1"},
  //       {"name": "comp2", "type": "type2"},
  //     ]
  //   },
  void createNodeFromJson(const nlohmann::json& node_json, const Prefix& prefix);

  // Starts the backend and all nodes which where added so far
  void start();
  // Stops the backend and all nodes
  void stop();
  // Destroys the backend and all nodes
  void destroy();

  // Starts a node and all its components
  void startNode(Node* node);
  // Stops a node and all its components
  void stopNode(Node* node);
  // Starts a list of nodes. Starting nodes multiple times is a warning.
  void startNodes(std::vector<Node*> nodes);
  void startNodes(std::initializer_list<Node*> nodes) {
    // TODO implement this more elegantly without a memory allocation
    startNodes(std::vector<Node*>{nodes});
  }
  // Stops a list of nodes nodes. Starting nodes multiple times is a warning.
  void stopNodes(const std::vector<Node*>& nodes);
  void stopNodes(std::initializer_list<Node*> nodes) {
    // TODO implement this more elegantly without a memory allocation
    stopNodes(std::vector<Node*>{nodes});
  }

  // Starts a codelet in case it was added after the node was already started. The codelet
  // will not be started in this thread.
  void requestCodeletStart(Codelet* codelet);

  // Adds a node/component status message update
  void reportStatusUpdate(StatusUpdate&& update);
  // Gets a list with all status updates and clears the list held by the backend.
  std::list<StatusUpdate> flushStatusUpdates();

  // Gets statistics about nodes
  nlohmann::json getNodeStatistics() const;
  // Gets statistics about codelets
  nlohmann::json getCodeletStatistics() const;

 private:
  // Finds the canister for a node. Asserts if the node is not known to the backend.
  NodeCanister* findCanister(Node* node) const;

  // Adds a task to the execution queue used for starting and stopping nodes.
  void addToStartStopQueue(std::function<void()> callback);

  // The main function for the execution queue used for starting and stopping nodes. Intended to
  // be executed by a blocking worker thread.
  void queueMain();
  // Performs one step of the execution queue.
  void processQueue();

  Application* app_ = nullptr;

  mutable std::mutex node_canisters_mutex_;
  std::set<std::string> node_names_;
  std::map<std::string, std::unique_ptr<NodeCanister>> node_canisters_;

  bool stop_requested_;

  mutable std::condition_variable queue_cv_;
  std::mutex queue_mutex_;
  std::optional<scheduler::JobHandle> job_handle_;
  std::vector<std::function<void()>> start_stop_queue_;
  std::atomic<bool> queue_running_;

  std::mutex status_update_queue_;
  std::list<StatusUpdate> status_updates_;
};

}  // namespace alice
}  // namespace isaac
