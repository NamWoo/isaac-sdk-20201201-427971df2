/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/backend/node_canister.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/codelet_canister.hpp"
#include "engine/alice/backend/modules.hpp"
#include "engine/alice/backend/names.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/component.hpp"
#include "engine/alice/components/Codelet.hpp"
#include "engine/alice/components/Config.hpp"
#include "engine/alice/components/Pose.hpp"
#include "engine/alice/components/Sight.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"

namespace isaac {
namespace alice {

namespace {

// Returns true if a component is a codelet
bool IsCodelet(Component* component) {
  return dynamic_cast<Codelet*>(component) != nullptr;
}

// Returns true if a component is a Config component
bool IsConfig(Component* component) {
  return dynamic_cast<Config*>(component) != nullptr;
}

}  // namespace

NodeCanister::NodeCanister(Application* app, std::string name)
    : app_(app) {
  ASSERT(app != nullptr, "Application must not be null");

  statistics_.num_started = 0;

  // Create a new node
  node_ = new Node(std::move(name));
  node_->canister_ = this;
  stage_ = LifecycleStage::kConstructed;
  app_->backend()->node_backend()->reportStatusUpdate({
    NowCount(), node()->name(), "", LifecycleStage::kNone, LifecycleStage::kConstructed
  });

  // Add components which are always available. Warning: the order of creation matters.
  node_->addComponent<Config>();
  node_->addComponent<Pose>();
  node_->addComponent<Sight>();
}

NodeCanister::~NodeCanister() {
  if (node_->getStage() != LifecycleStage::kDeinitialized) {
    LOG_WARNING("Node was not deinitialized before destruction");
  }
  delete node_;
}

Component* NodeCanister::addComponent(const std::string& type_name, std::string name) {
  // Assert that the name does not contain forbidden characters
  AssertValidName(name);

  // Assert that the name is unique within this node.
  ASSERT(node_->findComponentByName(name) == nullptr,
         "Component with name '%s' already exists in node", name.c_str());

  // Create the component
  Component* component = app_->backend()->module_manager()->createComponent(type_name);
  ASSERT(component != nullptr, "Failed to create component");

  // Wrap the component in a unique pointer
  std::unique_ptr<Component> uptr(component);
  uptr->name_ = std::move(name);
  uptr->uuid_ = Uuid::Generate();

  // Link the component to this node
  component->node_ = node_;

  // Connect all component hooks
  component->connectHooks();

  // We need special treatment for the config component.
  const bool is_config_component = IsConfig(component);
  Config* config = is_config_component ? static_cast<Config*>(component) : &node_->config();
  // In case this is the config component we need to initialize it first otherwise the backend
  // link will not be setup. In all other cases we need to intialize the component after we
  // have updated the hooks.
  if (is_config_component) {
    component->initialize();
  }
  // Set the type name of the component in the config.
  config->async_set(component, "__type_name", component->type_name());
  // Update configuration from root JSON
  config->updateHooks(component);
  // Make sure the config is aware of default parameters.
  config->updateCentralFromHooks(component);

  // Initialize components directly and codelets via the codelet canister
  const bool is_codelet = IsCodelet(component);
  if (is_codelet) {
    // Create a canister for the codelet which will also initialize it
    Codelet* codelet = static_cast<Codelet*>(component);
    std::unique_lock<std::mutex> lock(codelet_canisters_mutex_);
    codelet_canisters_.push_back(std::make_unique<CodeletCanister>(this, codelet));
  } else {
    // If this is a config component it was already initialized earlier.
    if (!is_config_component) {
      component->initialize();
    }
  }

  // Store the unique pointer
  // FIXME(dweikersdorf) It looks like it is not save to call addComponent concurrently. While this
  //                     should not happen we should protect properly against it.
  components_.emplace_back(std::move(uptr));

  // Depending on the lifecycle stage of the node some special actions might need to be taken.
  const LifecycleStage stage = getStage();
  if (stage == LifecycleStage::kConstructed) {
    // This is the most common case and does not required special treatment.
  } else if (stage == LifecycleStage::kPreStart) {
    // The component is being added while the node is starting. This is handled correctly due
    // to a careful implementation of startCodelets.
  } else if (stage == LifecycleStage::kStarted) {
    // The component is added to a node which is already running. If it is a codelet we need to
    // request the node backend to start the codelet.
    if (is_codelet) {
      app_->backend()->node_backend()->requestCodeletStart(static_cast<Codelet*>(component));
    }
  } else {
    PANIC("It is not allowed to add a component (typename: '%s', name: '%s') to a node (name '%s' "
          ") in its current state ('%d'). ", type_name.c_str(), component->name().c_str(),
          node_->name().c_str(), static_cast<int>(stage));
  }

  // Add the component to the node. From their on the component is discoverable by other nodes.
  node_->addComponent(component);

  return component;
}

Status NodeCanister::getStatus() const {
  std::unique_lock<std::mutex> lock(codelet_canisters_mutex_);
  Status combined = Status::SUCCESS;
  for (const auto& canister_uptr : codelet_canisters_) {
    combined = Combine(combined, canister_uptr->codelet()->getStatus());
  }
  return combined;
}

void NodeCanister::start() {
  if (node_->getStage() != LifecycleStage::kConstructed
      && node_->getStage() != LifecycleStage::kStopped) {
    LOG_WARNING("Can not start node '%s' because it is not in the stage 'constructed' or 'stopped'",
                node_->name().c_str());
    return;
  }

  stage_ = LifecycleStage::kPreStart;
  app_->backend()->node_backend()->reportStatusUpdate({
    NowCount(), node()->name(), "", LifecycleStage::kConstructed, LifecycleStage::kPreStart
  });
  statistics_.num_started++;

  startCodelets();

  stage_ = LifecycleStage::kStarted;
  app_->backend()->node_backend()->reportStatusUpdate({
    NowCount(), node()->name(), "", LifecycleStage::kPreStart, LifecycleStage::kStarted
  });
}

void NodeCanister::startCodeletImmediately(Codelet* codelet) const {
  ASSERT(codelet != nullptr, "Codelet must not be null");
  CodeletCanister* canister;
  {
    std::unique_lock<std::mutex> lock(codelet_canisters_mutex_);
    const auto it = std::find_if(codelet_canisters_.begin(), codelet_canisters_.end(),
                                 [codelet](const auto& uptr) {
                                   return uptr->codelet() == codelet;
                                 });
    ASSERT(it != codelet_canisters_.end(), "Codelet not found in node");
    canister = it->get();
  }

  // Information on why calling CodeletCanister::prepareToStart() is important before calling
  // CodeletCanister::start() can be found in NodeCanister::startCodelets().
  canister->prepareToStart(app_->backend()->scheduler());
  canister->start();
}

void NodeCanister::stop() {
  if (node_->getStage() == LifecycleStage::kStopped) {
    return;
  }
  if (node_->getStage() != LifecycleStage::kStarted) {
    LOG_WARNING("Can not stop node '%s' because it is not in the stage 'started'",
                node_->name().c_str());
    return;
  }

  stage_ = LifecycleStage::kPreStopped;
  app_->backend()->node_backend()->reportStatusUpdate({
    NowCount(), node()->name(), "", LifecycleStage::kStarted, LifecycleStage::kPreStopped
  });

  stopCodelets();

  stage_ = LifecycleStage::kStopped;
  app_->backend()->node_backend()->reportStatusUpdate({
    NowCount(), node()->name(), "", LifecycleStage::kPreStopped, LifecycleStage::kStopped
  });
}

void NodeCanister::deinitialize() {
  if (node_->getStage() != LifecycleStage::kStopped
      && node_->getStage() != LifecycleStage::kConstructed) {
    LOG_WARNING("Can not destruct node '%s' because it is not in the stage 'stopped' or ",
                "'constructed'", node_->name().c_str());
    return;
  }

  deinitializeComponents();

  stage_ = LifecycleStage::kDeinitialized;
}

nlohmann::json NodeCanister::getNodeStatistics() const {
  nlohmann::json json;
  json["lifecycle"] = node_->getStage();
  json["num_started"] = statistics_.num_started;
  return json;
}

nlohmann::json NodeCanister::getCodeletStatistics() const {
  std::unique_lock<std::mutex> lock(codelet_canisters_mutex_);
  nlohmann::json json;
  for (const auto& canister_uptr : codelet_canisters_) {
    json[canister_uptr->codelet()->name()] = canister_uptr->getStatistics();
  }
  return json;
}

void NodeCanister::setStatusForCodelets(Status status) {
  std::unique_lock<std::mutex> lock(codelet_canisters_mutex_);
  for (const auto& canister_uptr : codelet_canisters_) {
    canister_uptr->setCodeletStatus(status);
  }
}

std::vector<CodeletCanister*> NodeCanister::getCodeletCanisters() const {
  std::unique_lock<std::mutex> lock(codelet_canisters_mutex_);
  std::vector<CodeletCanister*> result;
  result.resize(codelet_canisters_.size());
  std::transform(codelet_canisters_.begin(), codelet_canisters_.end(), result.begin(),
                 [](const auto& uptr) { return uptr.get(); });
  return result;
}

void NodeCanister::startCodelets() {
  // Warning: Codelets might be added during the the Codelet start call. We thus have to continue
  // iterating until no more codelets are added. The `index` variable marks the index of the next
  // codelet canister to prepare and start.
  size_t index = 0;

  // CodeletCanister instances need to have `prepareToStart()` called before `start()`. This is the
  // case because already started codelets might make runtime decisions based on the status of other
  // codelets in the same node (which are then not started yet and still have their old status when
  // e.g. being restarted). This leads to unwanted race conditions. These effects don't take place
  // if all codelets are prepared first before starting any of them.

  while (true) {
    const std::vector<CodeletCanister*> canisters = getCodeletCanisters();
    if (index >= canisters.size()) {
      // No more CodeletCanister to start.
      return;
    }

    // Prepare all canisters. This also resets their status to `RUNNING` (rather than whatever
    // status they were in before).
    for (size_t start_index = index; start_index < canisters.size(); ++start_index) {
      canisters[start_index]->prepareToStart(app_->backend()->scheduler());
    }

    // Start all canisters. This actually starts the contained behaviors and adds the codelet to the
    // scheduler that was set during `prepareToStart()`.
    for (; index < canisters.size(); ++index) {
      canisters[index]->start();
    }
  }
}

void NodeCanister::stopCodelets() const {
  // Note that we need to iterate over a copy of the codelet canister list to avoid recursive
  // locking due to function calls in the user defined Codelet stop functions.
  const std::vector<CodeletCanister*> canisters = getCodeletCanisters();
  for (auto it = canisters.rbegin(); it != canisters.rend(); ++it) {
    (*it)->stop();
  }
}

void NodeCanister::deinitializeComponents() {
  // First all codelets
  {
    // Erase all codelet canisters which will also de-initialize the codelets.
    std::unique_lock<std::mutex> lock(codelet_canisters_mutex_);
    codelet_canisters_.clear();
  }
  // Second everything except codelets
  for (auto* component : node_->getComponents()) {
    if (!IsCodelet(component)) {
      component->deinitialize();
    }
  }
}

}  // namespace alice
}  // namespace isaac
