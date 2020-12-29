"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load(
    "@com_nvidia_isaac_engine//bzl:isaac_engine.bzl",
    "isaac_cc_library",
)
load(
    "//bzl:module.bzl",
    "isaac_cc_module",
    "isaac_component",
    "isaac_graph",
)
load("//doc:doc.bzl", "isaac_doc_dep")

filegroup(
    name = "webroot",
    data = glob(["webroot/**/*"]),
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "pose_tree_json_bridge",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/serialization:json",
    ],
)

isaac_cc_module(
    name = "sight",
    deps = [
        ":alice_sight",
        ":sight_start",
        ":websight_server",
    ],
)

isaac_cc_library(
    name = "sight_start",
    srcs = ["sight_start.cpp"],
    deps = [
        "//packages/sight:alice_sight",
        "//packages/sight:pose_tree_json_bridge",
        "@com_nvidia_isaac_engine//engine/alice:impl",
    ],
    alwayslink = True,
)

isaac_component(
    name = "sight_widget",
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/serialization:json",
    ],
)

isaac_component(
    name = "websight_server",
    data = [
        ":webroot",
        "@isaac_assets",
    ],
    deps = [
        "sight_widget",
        "@com_nvidia_isaac_engine//engine/gems/algorithm:string_utils",
        "@com_nvidia_isaac_engine//engine/gems/image",
        "@com_nvidia_isaac_engine//engine/gems/serialization:files",
        "@com_nvidia_isaac_engine//engine/gems/serialization:json",
        "@com_nvidia_isaac_engine//engine/gems/sight",
        "@uwebsockets",
    ],
)

isaac_component(
    name = "alice_sight",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/sight",
    ],
)

isaac_doc_dep(
    name = "doc",
    srcs = [
        "index.rst",
        "record_replay.rst",
    ] + glob(["doc/*"]),
    visibility = ["//visibility:public"],
)
