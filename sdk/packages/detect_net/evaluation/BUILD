"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_pkg")
load("//bzl:py.bzl", "isaac_py_app")

isaac_app(
    name = "detect_net_eval_inference",
    data = [
        "//packages/cask/apps:record_subgraph",
        "//packages/cask/apps:replay_subgraph",
        "//packages/detect_net/apps:config",
        "//packages/detect_net/apps:detect_net_inference",
    ],
    modules = [
        "sight",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "config",
    srcs = [
        "detect_net_cask_evaluation.config.json",
    ],
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "detect_net_cask_evaluation",
    srcs = [
        "detect_net_cask_evaluation.py",
    ],
    data = [
        "detect_net_evaluation_utils.py",
        ":config",
        "//packages/cask/apps:multi_cask_processing",
    ],
    modules = ["sight"],
)

isaac_py_app(
    name = "cvat_to_cask",
    srcs = [
        "cvat_to_cask.py",
    ],
    data = [
        "//packages/cask/apps:record_subgraph",
        "@detect_net_cvat_sample_data",
    ],
    modules = ["sight"],
    deps = [
        "//packages/pyalice",
    ],
)
