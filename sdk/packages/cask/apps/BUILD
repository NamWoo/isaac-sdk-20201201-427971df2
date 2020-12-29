"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_subgraph")
load("//bzl:py.bzl", "isaac_py_app")

py_binary(
    name = "analyse_cask",
    srcs = ["analyse_cask.py"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/pyalice",
    ],
)

py_binary(
    name = "record_small_point_clouds_test",
    srcs = ["record_small_point_clouds_test.py"],
    data = [
        "__init__.py",
        "//apps:py_init",
        "//apps/samples:py_init",
        "//packages/cask:libcask_module.so",
        "//packages/message_generators:libmessage_generators_module.so",
        "//packages/sight:libsight_module.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_subgraph(
    name = "record_subgraph",
    modules = [
        "cask",
        "sight",
    ],
    subgraph = "record.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "replay_subgraph",
    modules = [
        "cask",
        "sight",
    ],
    subgraph = "replay.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "cask_converter",
)

isaac_py_app(
    name = "multi_cask_processing",
    srcs = ["multi_cask_processing.py"],
    data = [
        "//packages/cask:libcask_module.so",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

py_binary(
    name = "write_speed_test",
    srcs = ["write_speed_test.py"],
    data = [
        "//packages/cask:libcask_module.so",
        "//packages/message_generators:libmessage_generators_module.so",
        "//packages/sight:libsight_module.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_py_app(
    name = "record_small_point_clouds_test-app",
    srcs = ["record_small_point_clouds_test"],
    main = "record_small_point_clouds_test.py",
)
