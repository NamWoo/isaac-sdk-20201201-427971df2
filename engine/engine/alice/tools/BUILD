"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "gather_component_info",
    srcs = ["gather_component_info.cpp"],
    hdrs = ["gather_component_info.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice:impl",
        "//engine/gems/serialization:json",
    ],
)

isaac_cc_library(
    name = "module",
    srcs = ["module.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":gather_component_info",
        "//engine/alice:impl",
    ],
    alwayslink = True,
)

isaac_cc_library(
    name = "parse_command_line",
    srcs = ["parse_command_line.cpp"],
    hdrs = ["parse_command_line.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":gather_component_info",
        "//engine/alice:impl",
        "//engine/gems/algorithm:string_utils",
        "//external:gflags",
    ],
)

isaac_cc_binary(
    name = "main",
    srcs = [
        "main.cpp",
    ],
    data = ["@redis//:redis-server"],
    visibility = ["//visibility:public"],
    deps = [
        ":parse_command_line",
        "//engine/alice:impl",
        "//external:gflags",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "check_json",
    srcs = ["check_json.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice:impl",
        "//engine/core",
        "//engine/gems/serialization:json",
    ],
)

exports_files(["run.sh.tpl"])

py_binary(
    name = "print_lifecycle_report",
    srcs = ["print_lifecycle_report.py"],
)
