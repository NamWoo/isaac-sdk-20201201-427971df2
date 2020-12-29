/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "path_utils.hpp"

#include <string>
#include <utility>

#include "absl/strings/str_format.h"

#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/algorithm/string_utils.hpp"

namespace isaac {
namespace alice {

namespace {
// Path to module libraries from external workspaces
constexpr const char kExternalPath[] = "external/";

// Prefix and suffix for workspace name
constexpr const char kWorkspacePrefix[] = "@";
constexpr size_t kWorkspacePrefixLength = sizeof(kWorkspacePrefix) - 1;  // '\0' does not count
constexpr const char kWorkspaceSuffix[] = "//";
constexpr size_t kWorkspaceSuffixLength = sizeof(kWorkspaceSuffix) - 1;  // '\0' does not count

// Expected folder name containing modules
constexpr const char kPackages[] = "packages";
constexpr size_t kPackagesLength = sizeof(kPackages) - 1;  // '\0' does not count
// Expected module naming convention
constexpr const char kModulePrefix[] = "lib";
constexpr const char kModuleSuffix[] = "_module.so";

// Extracts workspace name if present in path like @repo//
std::optional<std::string> ExtractWorkspace(const std::string path) {
  if (StartsWith(path, kWorkspacePrefix)) {
    size_t repo_end = path.find(kWorkspaceSuffix);
    ASSERT(repo_end != kWorkspacePrefixLength, "Invalid Repo Name");
    return repo_end == std::string::npos ? path.substr(kWorkspacePrefixLength)
                                         : path.substr(kWorkspacePrefixLength, repo_end - 1);
  }
  return std::nullopt;
}

// Expands module path (without workspace like @com_nvidia_isaac) into real path
std::string ExpandInWorkspaceModulePath(const std::string& raw_module) {
  std::string module = raw_module;

  // Remove leading `//`
  if (StartsWith(module, kWorkspaceSuffix)) {
    module = module.substr(2);
  }

  auto tokens = SplitString(module, ':');
  // First gets path and name sections
  std::string path, name;
  bool has_colon;
  if (tokens.size() == 1) {
    path = module;
    name = module;
    has_colon = false;
  } else if (tokens.size() == 2) {
    path = tokens[0];
    name = tokens[1];
    has_colon = true;
  } else {
    PANIC("Invalid module name '%s'", module.c_str());
  }
  // Expand the path to a full path
  tokens = SplitString(path, '/');
  if (tokens.size() == 1) {
    path = absl::StrFormat("%s/%s", kPackages, path);
  }
  if (!has_colon) {
    name = tokens.back();
  }

  return absl::StrFormat("%s/%s%s%s", path, kModulePrefix, name, kModuleSuffix);
}

}  // namespace

std::pair<std::string, std::string> TranslateAssetPath(const std::string& asset_path,
                                                       const std::string& home_workspace_name,
                                                       const std::string& context_workspace_name) {
  // In case of empty context workspace, assumes it is the same as home workspace
  const std::string& context_workspace =
      context_workspace_name.empty() ? home_workspace_name : context_workspace_name;

  std::string inner_path = asset_path;
  std::optional<std::string> workspace_name = ExtractWorkspace(asset_path);
  if (workspace_name) {
    // Removes workspace prefix like `@workspace//`
    inner_path =
        asset_path.substr(workspace_name->size() + kWorkspacePrefixLength + kWorkspaceSuffixLength,
                          std::string::npos);
    ASSERT(!inner_path.empty(), "Invalid asset path");
  } else {
    // Keep using the current context workspace name
    workspace_name = context_workspace;
  }

  std::string path = inner_path;

  const bool is_external_workspace = home_workspace_name.compare(*workspace_name) != 0;
  if (is_external_workspace) {
    path = absl::StrFormat("%s%s/%s", kExternalPath, *workspace_name, inner_path);
  }
  return {*workspace_name, path};
}

std::string ExpandModulePath(const std::string& module_target,
                             const std::string& home_workspace_name,
                             const std::string& context_workspace_name) {
  // In case of empty context workspace, assumes it is the same as home workspace
  const std::string& context_workspace =
      context_workspace_name.empty() ? home_workspace_name : context_workspace_name;

  std::string inner_path = module_target;
  std::optional<std::string> workspace_name = ExtractWorkspace(module_target);
  if (workspace_name) {
    // Removes workspace prefix like `@workspace`
    inner_path =
        module_target.substr(workspace_name->size() + kWorkspacePrefixLength, std::string::npos);
  } else {
    // Keep using the current context workspace name
    workspace_name = context_workspace;
  }

  std::string path = ExpandInWorkspaceModulePath(inner_path);

  const bool is_external_workspaces = home_workspace_name.compare(*workspace_name) != 0;
  if (is_external_workspaces) {
    path = absl::StrFormat("%s%s/%s", kExternalPath, *workspace_name, path);
  }

  return path;
}

std::string ExpandModulePath(const std::string& module_target) {
  return ExpandModulePath(module_target, "", "");
}

}  // namespace alice
}  // namespace isaac
