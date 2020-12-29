/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/tools/parse_command_line.hpp"

#include <string>

#include "absl/strings/str_split.h"
#include "engine/alice/tools/gather_component_info.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/uuid/uuid.hpp"
#include "gflags/gflags.h"

namespace isaac {
namespace alice {

DEFINE_string(app, "", "Filename of a JSON file which with the app definition.");
DEFINE_string(module_paths, "", "Paths to load module shared libs from, separated by comma.");
DEFINE_string(asset_path, "", "The path to search assets in.");
DEFINE_string(config, "",
              "A comma-separated list of configuration files in JSON format which "
              "specify additional configuration. Configuration parameters are written in the order "
              "in which they are loaded, thus the latest file takes precedences. Optionally, a "
              "prefix can be specified, which will be applied to all node names. The format is "
              "\"prefix_string:path_to_file.config.json\".");
DEFINE_string(graph, "",
              "A comma-separated list of node graph files in JSON format. This can be "
              "used to create additional nodes. Use with care as it might have unexpected side "
              " effects depending on which nodes are already loaded.");
DEFINE_string(more, "", "A comma-separated list of additional app files in JSON format to load.");
DEFINE_string(minidump_path, "/tmp", "Path to write minidump file in case of crash.");
DEFINE_string(application_backup, "",
              "Filename under which to store the whole application json "
              "just before the application is stopped.");
DEFINE_string(config_backup, "",
              "Filename under which to store the current configuration "
              "just before the application is stopped.");
DEFINE_string(performance_report_out, "",
              "Filename under which a performance report will be written just before the "
              "application is stopped.");
DEFINE_string(component_info_out, "",
              "If enabled a JSON object with information about all "
              "registered components is written to the specified file.");
DEFINE_string(locale, "", "Application locale.");

namespace {

ApplicationJsonLoader ParseApplicationCommandLineImpl(const std::optional<std::string>& name) {
  // Start by loading the application JSON file.
  nlohmann::json app_json;
  if (!FLAGS_app.empty()) {
    app_json = serialization::LoadJsonFromFile(FLAGS_app);
  }

  // If requested write component info to a file
  {
    auto it = app_json.find("component_info_out");
    if (it != app_json.end()) {
      const std::string component_info_out = *it;
      const bool success =
          serialization::WriteJsonToFile(component_info_out, GatherComponentInfo());
      if (!success) {
        LOG_ERROR("Failed to write component info to file '%s'", component_info_out);
      }
    }
  }

  // Potentially overwrite parameters in application JSON from command line
  if (!FLAGS_application_backup.empty()) {
    app_json["application_backup"] = FLAGS_application_backup;
  }
  if (!FLAGS_config_backup.empty()) {
    app_json["config_backup"] = FLAGS_config_backup;
  }
  if (!FLAGS_performance_report_out.empty()) {
    app_json["performance_report_out"] = FLAGS_performance_report_out;
  }
  if (!FLAGS_minidump_path.empty()) {
    app_json["minidump_path"] = FLAGS_minidump_path;
  }
  if (!FLAGS_locale.empty()) {
    app_json["locale"] = FLAGS_locale;
  }

  // Get application name
  std::string app_name;
  if (name) {
    // If an override is given we use it.
    app_json["name"] = *name;
  } else {
    // Try to get the name from the JSON
    const auto it = app_json.find("name");
    if (it == app_json.end()) {
      // Otherwise print a warning and use a random name
      const std::string random_name = Uuid::Generate().str();
      app_json["name"] = random_name;
      LOG_WARNING(
          "This application does not have a name! Please specify a name in the application JSON. "
          "A random name was used instead: %s",
          random_name.c_str());
    }
  }

  // Add config command line arguments to json
  if (!FLAGS_config.empty()) {
    if (app_json.find("config_files") == app_json.end()) {
      app_json["config_files"] = {};
    }
    auto filenames = SplitString(FLAGS_config, ',');
    for (const auto& config_file : filenames) {
      if (!config_file.empty()) {
        app_json["config_files"].push_back(config_file);
      }
    }
  }

  // Add graph command line arguments to json
  if (!FLAGS_graph.empty()) {
    if (app_json.find("graph_files") == app_json.end()) {
      app_json["graph_files"] = {};
    }
    auto filenames = SplitString(FLAGS_graph, ',');
    for (const auto& graph_file : filenames) {
      if (!graph_file.empty()) {
        app_json["graph_files"].push_back(graph_file);
      }
    }
  }

  // Create an instance of the application loader
  ApplicationJsonLoader loader(FLAGS_asset_path);

  // set module paths
  if (!FLAGS_module_paths.empty()) {
    loader.appendModulePaths(absl::StrSplit(FLAGS_module_paths, ","));
  }

  // load application
  loader.loadApp(app_json);

  // Load additional files
  if (!FLAGS_more.empty()) {
    for (const auto& filename : SplitString(FLAGS_more, ',')) {
      if (!filename.empty()) {
        const auto more_json = serialization::TryLoadJsonFromFile(filename);
        ASSERT(more_json, "Could not load additional file '%s'", filename.c_str());
        loader.loadMore(*more_json);
      }
    }
  }

  return loader;
}

}  // namespace

ApplicationJsonLoader ParseApplicationCommandLine() {
  return ParseApplicationCommandLineImpl(std::nullopt);
}

ApplicationJsonLoader ParseApplicationCommandLine(const std::string& name) {
  return ParseApplicationCommandLineImpl(name);
}

}  // namespace alice
}  // namespace isaac
