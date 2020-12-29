/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "application_json_loader.hpp"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <regex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/backend/message_ledger_backend.hpp"
#include "engine/alice/backend/names.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/utils/path_utils.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

namespace {

// Separates prefix from the filename. See GetFilenameAndPrefix() description for an example.
constexpr char kPrefixFilenameSeperator = ':';

// Parses input string and returns filename and prefix.
// If the input is "path_to_json.config.json", it returns
// { "path_to_json.config.json", Prefix() }
// If the input is "prefix:path_to_json.config.json", it returns
// { "path_to_json.config.json", Prefix("prefix") }
std::pair<std::string, Prefix> GetFilenameAndPrefix(const std::string& str) {
  auto tokens = SplitString(str, kPrefixFilenameSeperator);
  if (tokens.size() != 2) {
    // Prefix is not specified or the format is not as expected. Fall back to the old method.
    return {str, Prefix()};
  }
  return {tokens[1], Prefix(tokens[0])};
}

// Expands `$(fullname name)` values with prefix. For example, if the prefix is "navigation",
// "$(fullname shared_robot_model)" will be replaced with "navigation.shared_robot_model" in the
// given JSON object.
void ExpandFullnames(const Prefix& node_prefix, Json& json) {
  if (!json.is_object() && !json.is_array()) {
    return;
  }
  // Capture 3 groups:
  // First group is '$(fullname '.
  // Second group is the my_node_name, e.g., 'lqr' or 'joystick'.
  // Third group is the closing bracket ')'.
  const std::regex re("(\\$\\(fullname )(.*)(\\))");
  for (auto it = json.begin(); it != json.end(); ++it) {
    ExpandFullnames(node_prefix, *it);
    if (it->is_string()) {
      const std::string old_value = *it;
      // Second group, i.e. $2, captures the node name. Please see the definition of re above.
      const std::string new_value = std::regex_replace(old_value, re, node_prefix.apply("$2"));
      // If value has changed, update json
      if (new_value != old_value) {
        *it = new_value;
      }
    }
  }
}

// Given edge json and key value (currently "source" or "target"), reads the value.
// Applies the node prefix, unless user explicitly asked otherwise.
std::string ReadEdgeValue(const nlohmann::json& edge, const Prefix& node_prefix,
                          const std::string& key) {
  auto maybe_edge_value = serialization::TryGetFromMap<std::string>(edge, key);
  ASSERT(maybe_edge_value, "Missing mandatory field `%s` (type string) for edge value",
         key.c_str());
  std::string edge_value = *maybe_edge_value;
  // Special case: If an edge_value starts with special character '/', then we don't prefix it.
  if (edge_value.find("/") == 0) {
    // Delete the special character
    edge_value.erase(0, 1);
  } else {
    // Prefix the edge value
    edge_value = node_prefix.apply(edge_value);
  }
  return edge_value;
}

// Given a node name, return true if it's a default node that the application automatically adds
// for graph/config. This includes the backend, sight (for graph but not config) and nodes whose
// name starts with underscore. See engine/alice/tools/websight.cpp and
// engine/alice/backend/backend.cpp for details.
bool IsDefaultNode(const std::string& node, bool sight = true) {
  if (node.empty()) return true;
  if (node[0] == '_') return true;
  if (node == "backend") return true;
  if (sight && node == "websight") return true;
  return false;
}

// Given an edge, return true if it's a default edge that the application automatically adds to
// the graph. This is the case if the node is a default node (including sight).
bool IsDefaultEdge(const std::string& edge) {
  if (edge.empty()) return true;
  const auto separator_index = edge.find('/');
  if (separator_index == std::string::npos) return true;
  const std::string node = edge.substr(0, separator_index);
  return IsDefaultNode(node);
}

}  // namespace

ApplicationJsonLoader::ApplicationJsonLoader(const std::string& asset_path) : level_(0) {
  // Either use given asset path, or use current path.
  if (asset_path.empty()) {
    char* buffer = getcwd(nullptr, 0);
    asset_path_ = std::string(buffer);
    std::free(buffer);
  } else {
    asset_path_ = asset_path;
  }
}

std::string ApplicationJsonLoader::getAssetPath() const {
  return getAssetPath("", "");
}

std::string ApplicationJsonLoader::getAssetPath(const std::string& path) const {
  return getAssetPath(path, "");
}

std::string ApplicationJsonLoader::getAssetPath(const std::string& path,
                                                const std::string& workspace_name) const {
  // Do nothing about absolute path
  if (!path.empty() && path.at(0) == '/') {
    return path;
  }
  return asset_path_ + "/" + TranslateAssetPath(path, home_workspace_name_, workspace_name).second;
}

void ApplicationJsonLoader::setHomeWorkspaceName(const std::string& home_workspace_name) {
  home_workspace_name_ = home_workspace_name;
}

const std::string& ApplicationJsonLoader::getHomeWorkspaceName() {
  return home_workspace_name_;
}

void ApplicationJsonLoader::loadApp(const nlohmann::json& json) {
  // Get valid application name
  const auto it_name = json.find("name");
  if (it_name != json.end()) {
    const std::string app_name = *it_name;
    AssertValidName(app_name);
    name_ = app_name;
  } else {
    name_ = Uuid::Generate().str();
  }

  // Load home workspace name
  const auto it_workspace_name = json.find("workspace_name");
  if (it_workspace_name != json.end()) {
    home_workspace_name_ = *it_workspace_name;
  }

  // Backend config
  const auto it_backend = json.find("backend");
  if (it_backend != json.end()) {
    backend_json_ = *it_backend;
  } else {
    LOG_WARNING(
        "This application does not have an explicit scheduler configuration. "
        "One will be autogenerated to the best of the system's abilities if possible.");
    backend_json_ = {};
  }

  // Save parameters which we need when the application starts or shuts down
  const auto it_application_backup = json.find("application_backup");
  if (it_application_backup != json.end()) {
    application_backup_ = *it_application_backup;
  }
  const auto it_config_backup = json.find("config_backup");
  if (it_config_backup != json.end()) {
    config_backup_ = *it_config_backup;
  }
  const auto it_performance_report_out = json.find("performance_report_out");
  if (it_performance_report_out != json.end()) {
    performance_report_out_ = *it_performance_report_out;
  }
  const auto it_minidump_path = json.find("minidump_path");
  if (it_minidump_path != json.end()) {
    minidump_path_ = *it_minidump_path;
  }

  // Overrides to specific locale
  const auto it_locale = json.find("locale");
  if (it_locale != json.end()) {
    locale_ = *it_locale;
  }

  // the rest will happen via normal loading
  loadMore(json);
}

void ApplicationJsonLoader::loadModule(const std::string& module,
                                       const std::string& context_workspace) {
  modules_.push_back(
      ExpandModulePath(module, home_workspace_name_,
                       context_workspace.empty() ? home_workspace_name_ : context_workspace));
  module_names_.push_back(module);
}

void ApplicationJsonLoader::loadMore(const nlohmann::json& json, const Prefix& node_prefix,
                                     const std::string& context_workspace) {
  ASSERT(!json.is_null(), "Invalid JSON object");

  // modules
  const auto it_modules = json.find("modules");
  if (it_modules != json.end()) {
    std::vector<std::string> more_modules = *it_modules;
    for (auto& module : more_modules) {
      loadModule(module, context_workspace);
    }
  }

  // Load files that may have both graph and config
  const auto it_more_files = json.find("more_files");
  if (it_more_files != json.end()) {
    for (const auto& more_file : *it_more_files) {
      const auto filename_and_prefix = GetFilenameAndPrefix(more_file);
      loadMoreFromFile(filename_and_prefix.first,
                       node_prefix.apply_prefix(filename_and_prefix.second));
    }
  }

  // Load config
  const auto it_config = json.find("config");
  if (it_config != json.end()) {
    loadConfig(*it_config, node_prefix);
  }

  // Load config files
  const auto it_config_files = json.find("config_files");
  if (it_config_files != json.end()) {
    for (const auto& config_file : *it_config_files) {
      const auto filename_and_prefix = GetFilenameAndPrefix(config_file);
      loadConfigFromFile(filename_and_prefix.first,
                         node_prefix.apply_prefix(filename_and_prefix.second));
    }
  }

  // Load graph
  const auto it_graph = json.find("graph");
  if (it_graph != json.end()) {
    loadGraph(*it_graph, node_prefix, context_workspace);
  }

  // Load graph files
  const auto it_graph_files = json.find("graph_files");
  if (it_graph_files != json.end()) {
    for (const auto& graph_file : *it_graph_files) {
      loadGraphFromFile(graph_file, node_prefix);
    }
  }

  // Merge backend config if presents
  const auto it_backend = json.find("backend");
  if (it_backend != json.end()) {
    backend_json_ = serialization::JsonMerger().withJson(backend_json_).withJson(*it_backend);
  }

  if (it_modules == json.end() && it_config == json.end() && it_config_files == json.end() &&
      it_graph == json.end() && it_graph_files == json.end()) {
    LOG_WARNING("The Isaac app file did not contain any modules, config, or graph.");
  }
}

void ApplicationJsonLoader::loadMoreFromFile(const std::string& filename,
                                             const Prefix& node_prefix) {
  loadMore(serialization::LoadJsonFromFile(getAssetPath(filename, home_workspace_name_)),
           node_prefix);
}

void ApplicationJsonLoader::appendModulePaths(const std::vector<std::string>& module_paths) {
  module_paths_.insert(module_paths_.end(), module_paths.begin(), module_paths.end());
}

void ApplicationJsonLoader::loadConfig(const nlohmann::json& json, const Prefix& node_prefix) {
  // Add prefix to node names.
  nlohmann::json copy;
  for (auto it = json.begin(); it != json.end(); ++it) {
    copy[node_prefix.apply(it.key())] = *it;
  }
  // Expand '$(fullname <name>)' by appending prefix
  ExpandFullnames(node_prefix, copy);
  // merge by overwriting with new
  if (level_ >= config_by_level_.size()) {
    config_by_level_.resize(level_ + 1);
  }
  auto& config = config_by_level_[level_];
  config = serialization::MergeJson(config, copy);
}

void ApplicationJsonLoader::loadConfigFromFile(const std::string& filename,
                                               const Prefix& node_prefix) {
  loadConfig(serialization::LoadJsonFromFile(getAssetPath(filename, home_workspace_name_)),
             node_prefix);
}

void ApplicationJsonLoader::loadGraph(const nlohmann::json& json, const Prefix& node_prefix,
                                      const std::string& context_workspace) {
  level_ = 0;
  loadGraphRecursive(json, node_prefix,
                     context_workspace.empty() ? home_workspace_name_ : context_workspace);
  level_ = 0;
}

void ApplicationJsonLoader::loadGraphRecursive(const nlohmann::json& json,
                                               const Prefix& node_prefix,
                                               const std::string& workspace_name) {
  level_++;
  ASSERT(!json.is_null(), "Graph JSON must not be NULL");
  auto nodes_it = json.find("nodes");
  ASSERT(nodes_it != json.end(), "The `nodes` section is mandatory in the graph JSON");
  ASSERT(nodes_it->is_array(), "`nodes` JSON must be an array: %s", nodes_it->dump(2).c_str());
  for (auto it = nodes_it->begin(); it != nodes_it->end(); ++it) {
    const auto& json = *it;
    auto maybe_subgraph = serialization::TryGetFromMap<std::string>(json, "subgraph");
    if (maybe_subgraph) {
      // Retrieves corresponding workspace name for loading assets
      const std::string next_workspace_name =
          TranslateAssetPath(*maybe_subgraph, home_workspace_name_, workspace_name).first;

      auto maybe_name = serialization::TryGetFromMap<std::string>(json, "name");
      ASSERT(maybe_name, "Missing mandatory field 'name' (type string) for node");
      const Prefix nested_node_prefix(node_prefix.apply(*maybe_name));
      const std::string actual_subgraph_filename = getAssetPath(*maybe_subgraph, workspace_name);
      auto maybe_group_json = serialization::TryLoadJsonFromFile(actual_subgraph_filename);
      ASSERT(maybe_group_json, "Could not load subgraph from file '%s' (originally: '%s')",
             actual_subgraph_filename.c_str(), maybe_subgraph->c_str());
      // Load modules
      auto maybe_modules =
          serialization::TryGetFromMap<std::vector<std::string>>(*maybe_group_json, "modules");
      if (maybe_modules) {
        if (!maybe_modules->empty()) {
          // Only add modules which where not already loaded and create a smart info message.
          std::string text;
          for (size_t i = 0; i < maybe_modules->size(); i++) {
            const std::string& name = (*maybe_modules)[i];
            const std::string expanded_name =
                ExpandModulePath(name, home_workspace_name_, next_workspace_name);
            const bool is_new =
                std::find(modules_.begin(), modules_.end(), expanded_name) == modules_.end();
            if (is_new) {
              text += name;
              modules_.push_back(expanded_name);
              module_names_.push_back(name);
            } else {
              text += "(" + name + ")";
            }
            if (i + 1 < modules_.size()) {
              text += ", ";
            }
          }
          LOG_INFO("Modules requested by subgraph '%s': %s", maybe_name->c_str(), text.c_str());
        }
      } else {
        // TODO Check automatically if modules are missing via component namespace.
        LOG_WARNING("Loaded a subgraph with no modules section. Did you forget to add one?");
      }
      // Load config
      auto group_config = maybe_group_json->find("config");
      if (group_config != maybe_group_json->end()) {
        loadConfig(*group_config, nested_node_prefix);
      } else {
        LOG_WARNING("Loaded a subgraph with no config section. Did you forget to add one?");
      }
      // Load graph
      auto group_graph = maybe_group_json->find("graph");
      if (group_graph != maybe_group_json->end()) {
        loadGraphRecursive(*group_graph, nested_node_prefix, next_workspace_name);
      } else {
        LOG_WARNING("Loaded a subgraph with no graph section. Did you forget to add one?");
      }
    } else {
      nodes_.emplace_back(NodeJson{node_prefix, json});
      // add components
      for (auto& comp : json["components"]) {
        components_.insert(std::string(comp["type"]));
      }
    }
  }
  // setup connections:
  // "edges": [ {"source": "node/component/channel", "target": "node/component/channel"}, ... ]
  auto edges_it = json.find("edges");
  if (edges_it == json.end()) return;
  const nlohmann::json& edges = *edges_it;
  for (auto it = edges.begin(); it != edges.end(); ++it) {
    // Instead of connecting edges here, cache them since associated nodes may not be loaded yet.
    // We read all graphs first to get all the nodes. We'll connect edges before starting the
    // application.
    edges_.emplace_back(EdgeJson{ReadEdgeValue(*it, node_prefix, "source"),
                                 ReadEdgeValue(*it, node_prefix, "target")});
  }
  level_--;
}

void ApplicationJsonLoader::loadGraphFromFile(const std::string& filename,
                                              const Prefix& node_prefix) {
  loadGraph(serialization::LoadJsonFromFile(getAssetPath(filename, home_workspace_name_)),
            node_prefix);
}

void ApplicationJsonLoader::loadGraphFromText(const std::string& text, const Prefix& node_prefix) {
  loadGraph(serialization::LoadJsonFromText(text), node_prefix);
}

nlohmann::json ApplicationJsonLoader::GetGraphJson(Application& app, bool include_default) {
  nlohmann::json graph;
  graph["nodes"] = nlohmann::json::array();
  for (const Node* node : app.backend()->node_backend()->nodes()) {
    if (include_default || !IsDefaultNode(node->name())) {
      nlohmann::json json = ApplicationJsonLoader::GraphToJson(node->getComponents());
      json["name"] = node->name();
      if (node->disable_automatic_start) json["disable_automatic_start"] = true;
      if (node->start_order) json["start_order"] = node->start_order;
      graph["nodes"].emplace_back(std::move(json));
    }
  }
  graph["edges"] = nlohmann::json::array();
  for (const auto& connection : app.backend()->message_ledger_backend()->connections()) {
    if (include_default ||
        !(IsDefaultEdge(connection.source) || IsDefaultEdge(connection.target))) {
      graph["edges"].push_back({{"source", connection.source}, {"target", connection.target}});
    }
  }
  return graph;
}

nlohmann::json ApplicationJsonLoader::GetConfigJson(Application& app, bool include_default) {
  nlohmann::json config;
  for (const Node* node : app.backend()->node_backend()->nodes()) {
    if (include_default || !IsDefaultNode(node->name(), false)) {
      nlohmann::json json = ApplicationJsonLoader::ConfigToJson(node->getComponents());
      config[node->name()] = std::move(json[node->name()]);
    }
  }
  return config;
}

bool ApplicationJsonLoader::WriteJsonToFile(const std::string& filename, const Json& json) {
  std::ofstream ofs(filename);
  if (!ofs) {
    LOG_ERROR("Error opening file '%s'", filename.c_str());
    return false;
  }
  // Serialize graph JSON to text so that we can filter what we don't need
  const std::string text = json.dump(2);
  // Create a filter which removes `__type_name` key/value pairs and all empty dictionaries
  auto filter = [](int depth, auto event, Json& parsed) {
    if (parsed.is_string() && parsed == "__type_name") return false;
    if (parsed.is_object() && parsed.empty()) return false;
    return true;
  };
  // Filter JSON by parsing it again with the filter callback and then dump it to file
  ofs << Json::parse(text, filter, false).dump(2);
  return true;
}

nlohmann::json ApplicationJsonLoader::GraphToJson(const std::vector<Component*>& components) {
  nlohmann::json node_json;
  nlohmann::json component_list_json;
  for (Component* component : components) {
    nlohmann::json component_json;
    component_json["name"] = component->name();
    component_json["type"] = component->type_name();
    component_list_json.emplace_back(std::move(component_json));
  }
  node_json["components"] = std::move(component_list_json);
  return node_json;
}

nlohmann::json ApplicationJsonLoader::ConfigToJson(const std::vector<Component*>& components) {
  Json config_json;
  for (Component* component : components) {
    config_json[component->node()->name()][component->name()] =
        component->node()->config().getAll(component);
  }
  return config_json;
}

}  // namespace alice
}  // namespace isaac
