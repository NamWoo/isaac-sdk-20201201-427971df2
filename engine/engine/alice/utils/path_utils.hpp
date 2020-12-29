/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <utility>

namespace isaac {
namespace alice {

// Gets the real path of asset from a bazel target:
// In case of matching home_workspace and context_workspace,
//   foo                        => foo
//   path/foo                   => path/foo
//   //path/foo                 => path/foo
//   @repo//path/foo            => external/repo/path/foo/bar
//   @home_workspace//path/foo  => path/foo
// In case of non-matching home_workspace and context_workspace,
//   foo                        => external/context_workspace/foo
//   path/foo                   => external/context_workspace/path/foo
//   //path/foo                 => external/context_workspace/path/foo
//   @repo//path/foo            => external/repo/path/foo
//   @home_workspace//path/foo  => path/foo
//
// Returns:
//  pair<context_workspace_name/repo if present, real_asset_path >
std::pair<std::string, std::string> TranslateAssetPath(const std::string& asset_path,
                                                       const std::string& home_workspace_name,
                                                       const std::string& context_workspace_name);

// Gets the real path of a module (shared object file):
// In case of matching home_workspace and context_workspace,
//   foo                                => packages/foo/libfoo_module.so
//   //packages/foo                     => packages/foo/libfoo_module.so
//   //packages/foo:bar                 => packages/foo/libbar_module.so
//   @repo//packages/foo:bar            => external/repo/packages/foo/libbar_module.so
//   @home_workspace//packages/foo:bar  => external/repo/packages/foo/libbar_module.so
// In case of non-matching home_workspace and context_workspace,
//   foo                                => external/context_workspace/packages/foo/libfoo_module.so
//   //packages/foo                     => external/context_workspace/packages/foo/libfoo_module.so
//   //packages/foo:bar                 => external/context_workspace/packages/foo/libbar_module.so
//   @repo//packages/foo:bar            => external/repo/packages/foo/libbar_module.so
//   @home_workspace//packages/foo:bar  => external/repo/packages/foo/libbar_module.so
//
// Returns:
//  module_file_path
std::string ExpandModulePath(const std::string& module_target,
                             const std::string& home_workspace_name,
                             const std::string& context_workspace_name);
std::string ExpandModulePath(const std::string& module_target);

}  // namespace alice
}  // namespace isaac
