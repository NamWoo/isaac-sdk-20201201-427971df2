/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <set>
#include <string>
#include <vector>

#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

class Application;
class Component;

// Helper class to accumulate prefixes and apply them to node names
class Prefix {
 public:
  Prefix() = default;
  Prefix(const std::string& prefix_name) : prefix_name_(prefix_name) {}

  // Returns node name with prefix
  std::string apply(const std::string& node_name) const {
    if (prefix_name_.empty()) {
      return node_name;
    }
    return prefix_name_ + delimiter_ + node_name;
  }

  // Creates new prefix by putting one under current prefix
  Prefix apply_prefix(const Prefix& sub_prefix) const {
    if (prefix_name_.empty()) {
      return sub_prefix;
    }
    if (sub_prefix.prefix_name_.empty()) {
      return *this;
    }
    return Prefix(this->apply(sub_prefix.prefix_name_));
  }

 private:
  // The prefix that is accumulated using delimiters. It does not have a trailing delimiter.
  std::string prefix_name_;
  // Delimiter to use when concatenating prefixes or when applying prefix name to node names.
  const std::string delimiter_ = ".";
};

// Helper class to load applications from JSON
class ApplicationJsonLoader {
 public:
  ApplicationJsonLoader(const std::string& asset_path = "");

  // Gets absolute path for assets file from specified workspace. Empty workspace name implies
  // current workspace. Identity in case of absolute filename.
  std::string getAssetPath(const std::string& path, const std::string& workspace_name) const;
  // Gets absolute path for assets file from current workspace. Identity in case of absolute
  // filename.
  std::string getAssetPath(const std::string& path) const;
  // Gets absolute folder path containing assets in current workspace.
  std::string getAssetPath() const;

  // Sets the name of current home workspace for proper handling of external assets
  void setHomeWorkspaceName(const std::string& home_workspace_name);
  // Returns the name of current home workspace for proper handling of external assets
  const std::string& getHomeWorkspaceName();

  // Loads an application from a JSON object
  void loadApp(const nlohmann::json& json);

  // Loads more config and graph from a JSON object
  void loadMore(const nlohmann::json& json, const Prefix& node_prefix = Prefix(),
                const std::string& context_workspace = "");
  // Loads JSON from a file and then proceeds like `loadMore` for JSON objects.
  void loadMoreFromFile(const std::string& filename, const Prefix& node_prefix = Prefix());

  // Loads one module explicitly
  void loadModule(const std::string& module, const std::string& context_workspace = "");

  // Adds more paths where to look for modules
  void appendModulePaths(const std::vector<std::string>& module_paths);

  // Loads configuration from the given JSON object. `node_prefix` can be used to add a string
  // prefix to all node names.
  void loadConfig(const nlohmann::json& json, const Prefix& node_prefix = Prefix());
  // Loads JSON from a file and then proceeds like `loadConfig` for JSON objects. `node_prefix` can
  // be used to add a string prefix to all node names.
  void loadConfigFromFile(const std::string& filename, const Prefix& node_prefix = Prefix());

  // Loads a graph from the given JSON object. `node_prefix` can be used to add a string prefix
  // to all node names.
  void loadGraph(const nlohmann::json& json, const Prefix& node_prefix = Prefix(),
                 const std::string& context_workspace = "");
  // Loads JSON from a file and then proceeds like `loadGraph` for JSON objects.
  void loadGraphFromFile(const std::string& filename, const Prefix& node_prefix = Prefix());
  // Loads JSON from a string and then proceeds like `loadGraph` for JSON objects.
  void loadGraphFromText(const std::string& text, const Prefix& node_prefix = Prefix());

  // Gets a list of names of all modules in the app including it's subgraphs
  std::vector<std::string> getModuleNames() const { return modules_; }
  // Gets a set of names of all components in the app including it's subrgaphs
  std::set<std::string> getComponentNames() const { return components_; }

  // Gets all nodes and edges as JSON. If include_default is true, the output includes nodes that
  // are added automatically.
  static nlohmann::json GetGraphJson(Application& app, bool include_default = true);
  // Gets all configurations as JSON. If include_default is true, the output includes nodes that
  // are added automatically.
  static nlohmann::json GetConfigJson(Application& app, bool include_default = true);

  // Gets graph json for all the given components
  static nlohmann::json GraphToJson(const std::vector<Component*>& components);
  // Gets configuration json for all the given components
  static nlohmann::json ConfigToJson(const std::vector<Component*>& components);

  // Writes given json (application, graph or config) to a file with basic filtering
  static bool WriteJsonToFile(const std::string& filename, const Json& json);

 private:
  friend class Application;

  // Information about a node in JSON format
  struct NodeJson {
    // Prefix associated with this node
    Prefix prefix;
    // The JSON object describing the nodes, e.g. it's components
    nlohmann::json json;
  };

  // Information about a message connection
  struct EdgeJson {
    std::string source;
    std::string target;
  };

  // Helper function for loadGraph
  void loadGraphRecursive(const nlohmann::json& json, const Prefix& node_prefix,
                          const std::string& workspace_name);

  std::string asset_path_;

  std::string name_;
  std::string application_backup_;
  std::string config_backup_;
  std::string performance_report_out_;
  std::string minidump_path_;

  nlohmann::json backend_json_;

  std::vector<std::string> module_paths_;
  // stores the expanded module names
  std::vector<std::string> modules_;
  // stores the unexpanded modules as is, for application_backup
  std::vector<std::string> module_names_;

  // Name of components
  std::set<std::string> components_;

  // Name of workspace that the app lives in
  std::string home_workspace_name_;

  size_t level_;
  std::vector<nlohmann::json> config_by_level_;

  // Loaded nodes
  std::vector<NodeJson> nodes_;

  // Loaded edges
  std::vector<EdgeJson> edges_;

  // Overrides to specific locale
  std::string locale_;
};

}  // namespace alice
}  // namespace isaac
