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
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "engine/alice/node.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace alice {

class ApplicationJsonLoader;
class Backend;
class PybindApplication;

// The basis of an Isaac application. An application normally contains many nodes which work
// together to create complex behaviors. A node in turn contains multiple components which define
// its functionality. Components can be configured via parameters, exchange data via messages, and
// much more.
//
// Applications can be created as specified in a JSON object. They can also be built interactively
// at runtime, however not all features are yet supported for this use case.
//
// If you want to create an application which parses basic parameters via the command line you
// can use the //engine/alice/tools:parse_command_line. This is kept as a separate library to not
// pollute the default compile object with static variable from gflags.
//
// The API of this class is still under development and should considered to be experimental.
class Application {
 public:
  // Creates a new application
  Application(const ApplicationJsonLoader& loader);
  // Creates a new application with a random name
  Application(const std::vector<std::string> module_paths = {}, const std::string& asset_path = "");
  // Creates a new application as specified in the given JSON object.
  Application(const nlohmann::json& json, const std::vector<std::string> module_paths = {},
              const std::string& asset_path = "");

  ~Application();

  // A name for the app which stays the same over time
  const std::string& name() const { return name_; }
  // A unique identifier which is different for every running application
  const Uuid& uuid() const { return uuid_; }

  // Loads more configuration and nodes from a JSON file
  void loadFromFile(const std::string& json_file);
  // Loads more configuration and nodes from a JSON text string
  void loadFromText(const std::string& json_text);
  // Loads more configuration and nodes from a JSON object
  void load(const nlohmann::json& json);

  // Loads a module
  void loadModule(const std::string& name);

  // Creates a new node with the given name
  Node* createNode(const std::string& name);
  Node* createMessageNode(const std::string& name);
  // Destroys the node with the given name and all its components
  void destroyNode(const std::string& name);

  // Finds a node by name. This function will return nullptr if no node with this name exists.
  Node* findNodeByName(const std::string& name) const;
  // Gets a node by name. This function will assert if no node with this name exists.
  Node* getNodeByName(const std::string& name) const;

  // Find all nodes which have a component of the given type
  template <typename T>
  std::vector<Node*> findNodesWithComponent() const {
    std::vector<Node*> result;
    for (Node* node : nodes()) {
      if (node->hasComponent<T>()) {
        result.push_back(node);
      }
    }
    return result;
  }
  // Find all components of the given type. This function is quite slow.
  template <typename T>
  std::vector<T*> findComponents() const {
    std::vector<T*> result;
    for (Node* node : nodes()) {
      const auto components = node->getComponents<T>();
      result.insert(result.end(), components.begin(), components.end());
    }
    return result;
  }
  // Finds a unique component of the given type. Returns null if none or multiple components of this
  // type where found. This function is quite slow.
  template <typename T>
  T* findComponent() const {
    T* pointer = nullptr;
    for (Node* node : nodes()) {
      if (T* component = node->getComponentOrNull<T>()) {
        if (pointer != nullptr) {
          return nullptr;
        }
        pointer = component;
      }
    }
    return pointer;
  }
  // Similar to `findComponent` but fails in case it could not find a com
  template <typename T>
  T* getComponent() const {
    T* result = findComponent<T>();
    ASSERT(result, "Could not find a unique component with given type.");
    return result;
  }

  // @deprecated: Use `getNodeComponentOrNull` instead.
  // Find a component by name and type. `link` is given as "node_name/component_name"
  template <typename T>
  T* findComponentByName(const std::string& link) const {
    return dynamic_cast<T*>(findComponentByName(link));
  }
  // Find a component by name. `link` is given as "node_name/component_name"
  Component* findComponentByName(const std::string& link) const;

  // Gets the component of given type in the node of given name. Asserts in case the node
  // does not exists, or if there are none or multiple components of the given type in the node.
  template <typename T>
  T* getNodeComponent(const std::string& node_name) const {
    return getNodeByName(node_name)->getComponent<T>();
  }

  // Gets the component of given type in the node of given name. nullptr is returned in case the
  // node does not exist, or if there are none or multiple components of the given type in the node.
  template <typename T>
  T* getNodeComponentOrNull(const std::string& node_name) const {
    const Node* node = findNodeByName(node_name);
    if (node == nullptr) return nullptr;
    return node->getComponentOrNull<T>();
  }

  // For a channel 'nodename/compname/tag', return the component and tag string
  std::tuple<Component*, std::string> getComponentAndTag(const std::string& channel);

  // Connects the source channel to the target channel. Both strings must be valid channel tags
  // in the format "node_name/component_name/channel_name".
  void connect(const std::string& source, const std::string& target);

  // Sets a timeout for the application. If not interrupted, the application will stop automatically
  // after the timeout expire, the time will start when the application starts.
  // If a negative time is provided, the timeout will be removed.
  void enableStopOnTimeout(double timeout);

  // The application will be automatically stopped if the provided node is stopped.
  void enableStopOnNode(const std::string node_name);

  // Disables the application to stop when control + C is called.
  void disableStopOnCtrlC();

  // Enables the application to stop when control + C is called.
  void enableStopOnCtrlC();

  // Starts the application and returns right away.
  void runAsync();

  // Starts the application and waits for the application to be stopped.
  void runBlocking();

  // Request the application to stop. This function will return directly. It can be called from
  // anywhere, including from a Codelet.
  void stopAsync();

  // Request the application to stop and wait for. It can be called from
  // anywhere, including from a Codelet.
  void stopBlocking();

  Backend* backend() const { return backend_.get(); }

  // Gets absolute filename for relative asset filename. Identity in case of absolute filename.
  std::string getAssetPath(const std::string& path = "") const;

  // Creates more configuration and graph
  void createMore(const ApplicationJsonLoader& loader);

 private:
  friend class Node;
  friend class Backend;
  friend class PybindApplication;

  // Starts the application's backends
  void startImpl();
  // Srops the application's backends
  void stopImpl();

  // Gets all nodes
  std::vector<Node*> nodes() const;

  // Creates a new application from the given JSON object.
  void createApplication(const ApplicationJsonLoader& loader);

  std::string name_;
  Uuid uuid_;
  std::unique_ptr<Backend> backend_;

  std::atomic<bool> is_running_;

  // Filename to write out the application json
  std::string application_backup_;
  // Cache the application json while the app is loading to write out to file later
  nlohmann::json app_json_;

  // Filename to write out the configuration
  std::string config_backup_;
  // Filename to write out the performance report
  std::string performance_report_out_;

  // Base path to search assets for
  std::string asset_path_;

  // Used to check for case of starting twice
  bool started_ = false;
  // Used to check for case of stoping twice
  bool stopped_ = false;
  // Thread used to run the application.
  std::thread application_thread_;
  std::thread::id application_thread_id_;

  std::set<std::string> stop_on_nodes_;
  std::optional<double> timeout_ = std::nullopt;
};

}  // namespace alice
}  // namespace isaac
