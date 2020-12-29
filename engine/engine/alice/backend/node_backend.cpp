/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "node_backend.hpp"

#include <algorithm>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/component_registry.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/backend/names.hpp"
#include "engine/alice/backend/node_canister.hpp"
#include "engine/alice/component.hpp"
#include "engine/alice/components/Codelet.hpp"
#include "engine/alice/components/Config.hpp"
#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/components/Pose.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/scheduler/scheduler.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

namespace {

// The duration for which the execution queue conditional variable waits at most before running
// the next procesing step.
constexpr std::chrono::seconds kMaxEmptyQueueWaitTime = std::chrono::seconds(1);

}  // namespace

NodeBackend::NodeBackend(Application* app) : app_(app) {}

NodeBackend::~NodeBackend() {}

Node* NodeBackend::createNode(const std::string& name) {
  // make sure that the name does not contains forbidden characters
  AssertValidName(name);

  std::lock_guard<std::mutex> lock(node_canisters_mutex_);

  // make sure the node does not exist yet
  const auto result = node_names_.insert(name);
  ASSERT(result.second, "A node with name '%s' already exists.", name.c_str());

  // Create a canister with a new node
  auto canister_ptr = std::make_unique<NodeCanister>(app_, name);
  Node* node = canister_ptr->node();
  node_canisters_.insert({name, std::move(canister_ptr)});

  return node;
}

Node* NodeBackend::createMessageNode(const std::string& name) {
  Node* node = createNode(name);
  node->addComponent<MessageLedger>();
  return node;
}

void NodeBackend::destroyNode(const std::string& name) {
  NodeCanister* canister;
  {
    std::lock_guard<std::mutex> lock(node_canisters_mutex_);
    // Find the canister of the node
    const auto it = node_canisters_.find(name);
    ASSERT(it != node_canisters_.end(), "Node with name '%s' does not exist");
    // We need to do a bit of an unusual operation here. We want to remove the unique pointer
    // holding the node from the node list. Ideally we would like to give this pointer to our lambda
    // below. However C++ standard does not allow to move a non-copyable lambda into an
    // std::function. Thus we release the unique pointer here and manually delete the node in the
    // lambda below.
    canister = it->second.get();
    it->second.release();
    // Remove the node from the nodes list. This means that the node is not discoverable anymore.
    // The lambda we create below will take care of the remaining steps to destroy the node.
    node_canisters_.erase(it);
  }
  // Add a job to destroy the job.
  std::lock_guard<std::mutex> lock(queue_mutex_);
  start_stop_queue_.emplace_back([canister] {
    // Stop and destroy the canister and the node it contains
    canister->stop();
    canister->deinitialize();
    delete canister;
  });
}

void NodeBackend::createNodeFromJson(const nlohmann::json& node_json, const Prefix& prefix) {
  static const std::vector<std::string> kDefaultComponents{
      "isaac::alice::Config", "isaac::alice::Pose", "isaac::alice::Sight"};
  static const std::string kMessageLeger = "isaac::alice::MessageLedger";
  static const std::string kNodeGroup = "isaac::behavior_tree::NodeGroup";
  // get node name
  auto maybe_name = serialization::TryGetFromMap<std::string>(node_json, "name");
  ASSERT(maybe_name, "Missing mandatory field 'name' (type string) for node");
  const std::string node_name = prefix.apply(*maybe_name);
  // create a new node
  Node* node = createNode(node_name);
  // set disable_automatic_start if present in JSON
  const auto disable_automatic_start = app()->backend()->config_backend()->
      getAllForComponent(node_name, "disable_automatic_start");
  if (!disable_automatic_start.empty()) {
    // if set in config use this
    node->disable_automatic_start = disable_automatic_start;
  } else {
    // otherwise use check if set in graph
    auto disable_automatic_start_it = node_json.find("disable_automatic_start");
    if (disable_automatic_start_it != node_json.end()) {
      node->disable_automatic_start = *disable_automatic_start_it;
    }
  }

  // set start_order if present in JSON
  auto start_order_it = node_json.find("start_order");
  if (start_order_it != node_json.end()) {
    node->start_order = *start_order_it;
  }
  // find components to add
  auto node_components_json = node_json["components"];
  ASSERT(!node_components_json.is_null(), "Node must have entry 'components'");
  std::map<std::string, std::string> components_to_add;
  std::optional<std::string> message_ledger;
  std::optional<std::string> node_group;
  for (const auto& cjson : node_components_json) {
    // get the name
    auto name_json = cjson["name"];
    ASSERT(name_json.is_string(), "Component entry must contain 'name' of type string: %s",
           cjson.dump(2).c_str());
    const std::string name = name_json;
    // get the type
    auto type_json = cjson["type"];
    ASSERT(type_json.is_string(), "Component entry must contain 'type' of type string: %s",
           cjson.dump(2).c_str());
    const std::string type = type_json;
    // Don't add mandatory components.
    if (std::find(kDefaultComponents.begin(), kDefaultComponents.end(), type) !=
        kDefaultComponents.end()) {
      LOG_WARNING("Default components should not be explicitly specified: node = %s, "
                  "component = %s", node_name.c_str(), name.c_str());
      continue;
    }
    // Is it a message ledger?
    if (type == kMessageLeger) {
      ASSERT(!message_ledger, "Can only add one message ledger per node");
      message_ledger = name;
      continue;
    }
    // Is it a node group?
    if (type == kNodeGroup) {
      ASSERT(!node_group, "Can only add one node group per node");
      node_group = name;
      continue;
    }
    // Store for later
    auto it = components_to_add.find(name);
    ASSERT(it == components_to_add.end(), "Component name must be unique '%s'", name.c_str());
    components_to_add[name] = type;
  }
  // create components. make sure to create message ledger first
  if (message_ledger) {
    node->addComponent(kMessageLeger, *message_ledger);
  }
  if (node_group) {
    node->addComponent(kNodeGroup, *node_group);
  }
  for (const auto& kvp : components_to_add) {
    node->addComponent(kvp.second, kvp.first);
  }
}

Node* NodeBackend::findNodeByName(const std::string& name) const {
  std::lock_guard<std::mutex> lock(node_canisters_mutex_);
  const auto it = node_canisters_.find(name);
  return it == node_canisters_.end() ? nullptr : it->second->node();
}

size_t NodeBackend::numNodes() const {
  std::lock_guard<std::mutex> lock(node_canisters_mutex_);
  return node_canisters_.size();
}

std::vector<Node*> NodeBackend::nodes() const {
  std::vector<Node*> result;
  result.reserve(node_canisters_.size());
  std::lock_guard<std::mutex> lock(node_canisters_mutex_);
  for (const auto& kvp : node_canisters_) {
    result.push_back(kvp.second->node());
  }
  return result;
}

void NodeBackend::start() {
  stop_requested_ = false;
  queue_running_ = true;

  // ToDo: If there is not a default blocker group this will fail.
  scheduler::JobDescriptor job_descriptor;
  job_descriptor.priority = 0;
  job_descriptor.execution_mode = scheduler::ExecutionMode::kBlockingOneShot;
  job_descriptor.name = "NodeBackend start/stop queue";
  job_descriptor.action = [this] { queueMain(); };
  job_handle_ = app()->backend()->scheduler()->createJobAndStart(job_descriptor);
  ASSERT(job_handle_, "Unable to create Node Queue Processing Job");

  // Sort nodes by start order
  std::lock_guard<std::mutex> lock1(node_canisters_mutex_);
  // Start all nodes for which automatic start is not disabled
  std::lock_guard<std::mutex> lock2(queue_mutex_);
  // First get the list of node to start
  std::vector<NodeCanister*> node_to_start;
  for (auto& kvp : node_canisters_) {
    NodeCanister* canister = kvp.second.get();
    if (canister->node()->disable_automatic_start) {
      continue;
    }
    node_to_start.push_back(canister);
  }
  // Sort them using start_order
  std::stable_sort(node_to_start.begin(), node_to_start.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs->node()->start_order < rhs->node()->start_order;
      });
  // Start the nodes
  for (auto& canister : node_to_start) {
    addToStartStopQueue([canister] { canister->start(); });
  }
}

void NodeBackend::stop() {
  stop_requested_ = true;
  {
    // TODO Can not use stopNodes here because we have unique_ptr in nodes_
    std::lock_guard<std::mutex> lock1(node_canisters_mutex_);
    std::lock_guard<std::mutex> lock2(queue_mutex_);
    // Stop nodes in reverse order as a best effort
    for (auto it = node_canisters_.rbegin(); it != node_canisters_.rend(); ++it) {
      NodeCanister* canister = it->second.get();
      if (canister->node()->disable_automatic_start) {
        continue;
      }
      addToStartStopQueue([canister] { canister->stop(); });
    }
  }
  queue_running_ = false;
  // Extra notify for the case where no nodes are being destroyed at shutdown.
  queue_cv_.notify_all();
  app()->backend()->scheduler()->destroyJobAndWait(*job_handle_);
}

void NodeBackend::destroy() {
  std::lock_guard<std::mutex> lock(node_canisters_mutex_);

  // Deinitialize all nodes
  for (auto it = node_canisters_.rbegin(); it != node_canisters_.rend(); ++it) {
    it->second->deinitialize();
  }

  // Destroys all node canisters and the nodes they contain
  node_canisters_.clear();
}

void NodeBackend::startNode(Node* node) {
  if (stop_requested_) {
    LOG_WARNING("Node '%s' not started because NodeBackend is shutting down", node->name().c_str());
    return;
  }
  NodeCanister* canister = findCanister(node);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    addToStartStopQueue([canister] { canister->start(); });
  }
}

void NodeBackend::stopNode(Node* node) {
  NodeCanister* canister = findCanister(node);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    addToStartStopQueue([canister] { canister->stop(); });
  }
}

void NodeBackend::startNodes(std::vector<Node*> nodes) {
  // Reset all codelets' status in each node to RUNNING to prevent race conditions during their
  // initialization.
  for (Node* node : nodes) {
    findCanister(node)->setStatusForCodelets(Status::RUNNING);
  }
  for (Node* node : nodes) {
    startNode(node);
  }
}

void NodeBackend::stopNodes(const std::vector<Node*>& nodes) {
  for (Node* node : nodes) {
    stopNode(node);
  }
}

void NodeBackend::requestCodeletStart(Codelet* codelet) {
  ASSERT(codelet != nullptr, "Component must not be null");
  NodeCanister* canister = findCanister(codelet->node());
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    addToStartStopQueue([canister, codelet] { canister->startCodeletImmediately(codelet); });
  }
}

void NodeBackend::reportStatusUpdate(StatusUpdate&& update) {
  std::lock_guard<std::mutex> lock(status_update_queue_);
  status_updates_.emplace_back(std::move(update));
}

std::list<NodeBackend::StatusUpdate> NodeBackend::flushStatusUpdates() {
  std::lock_guard<std::mutex> lock(status_update_queue_);
  auto copy = std::move(status_updates_);
  status_updates_ = {};
  return copy;
}

nlohmann::json NodeBackend::getNodeStatistics() const {
  std::lock_guard<std::mutex> lock(node_canisters_mutex_);
  nlohmann::json json;
  for (const auto& kvp : node_canisters_) {
    json[kvp.first] = kvp.second->getNodeStatistics();
  }
  return json;
}

nlohmann::json NodeBackend::getCodeletStatistics() const {
  std::lock_guard<std::mutex> lock(node_canisters_mutex_);
  nlohmann::json json;
  for (const auto& kvp : node_canisters_) {
    json[kvp.first] = kvp.second->getCodeletStatistics();
  }
  return json;
}

NodeCanister* NodeBackend::findCanister(Node* node) const {
  ASSERT(node != nullptr, "Node must not be null");
  std::lock_guard<std::mutex> lock(node_canisters_mutex_);
  const auto it = node_canisters_.find(node->name());
  ASSERT(it != node_canisters_.end(), "Node is not known to this backend");
  return it->second.get();
}

void NodeBackend::addToStartStopQueue(std::function<void()> callback) {
  start_stop_queue_.push_back(callback);
  queue_cv_.notify_all();
}

void NodeBackend::queueMain() {
  while (queue_running_) {
    processQueue();
    std::unique_lock<std::mutex> lock(queue_mutex_);
    while (queue_running_ && start_stop_queue_.empty()) {
      // limit the amount of time we wait, so we can recheck the queue running flag
      queue_cv_.wait_for(lock, kMaxEmptyQueueWaitTime);
    }
  }
  // At this point all globally governed nodes are in the stop queue. However other nodes might
  // only be stopped during the stop of another node. So we iterate until the start/stop queue
  // is really empty.
  while (true) {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if (start_stop_queue_.empty()) {
        break;
      }
    }
    processQueue();
  }
}

void NodeBackend::processQueue() {
  std::vector<std::function<void()>> copy;
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    copy = std::move(start_stop_queue_);
    start_stop_queue_ = {};
  }
  for (auto& task : copy) {
    task();
  }
}

}  // namespace alice
}  // namespace isaac
