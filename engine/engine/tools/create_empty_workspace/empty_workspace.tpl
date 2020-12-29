'''
Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
workspace(name = "empty_workspace")

local_repository(
    name = "com_nvidia_isaac_engine",
    path = "$isaac_path/engine",
)

local_repository(
    name = "com_nvidia_isaac",
    path = "$isaac_path/sdk",
)


load("@com_nvidia_isaac_engine//bzl:deps.bzl", "isaac_git_repository")
load("@com_nvidia_isaac_engine//third_party:engine.bzl", "isaac_engine_workspace")

load("@com_nvidia_isaac//third_party:packages.bzl", "isaac_packages_workspace")


isaac_engine_workspace()

isaac_packages_workspace()

# Loads boost c++ library (https://www.boost.org/) and
# custom bazel build support (https://github.com/nelhage/rules_boost/) explicitly
# due to bazel bug: https://github.com/bazelbuild/bazel/issues/1550
isaac_git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "82ae1790cef07f3fd618592ad227fe2d66fe0b31",
    licenses = ["@com_github_nelhage_rules_boost//:LICENSE"],
    remote = "https://github.com/nelhage/rules_boost.git",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")

boost_deps()

# Configures toolchain
load("@com_nvidia_isaac_engine//toolchain:toolchain.bzl", "toolchain_configure")

toolchain_configure(name = "toolchain")