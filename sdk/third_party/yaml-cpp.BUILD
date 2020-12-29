"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "yaml-cpp",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.h",
    ]),
    hdrs = glob([
        "include/**/*.h",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)

# Lula requires yaml-cpp as a shared library with this exact name.
cc_binary(
    name = "libyaml-cpp.so.0.6",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.h",
        "include/**/*.h",
    ]),
    includes = ["include"],
    linkshared = True,
)

cc_import(
    name = "libyaml-cpp.so",
    shared_library = "libyaml-cpp.so.0.6",
    visibility = ["//visibility:public"],
)
