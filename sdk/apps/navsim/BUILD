"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app")
load("//bzl:py.bzl", "isaac_py_app")

isaac_py_app(
    name = "navsim_navigate",
    srcs = [
        "navsim_navigate.py",
    ],
    data = [
        "navsim_navigate.app.json",
        "//apps/assets/maps",
        "//packages/behavior_tree/apps:missions",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "behavior_tree",
        "map",
        "sight",
        "viewers",
    ],
)

isaac_app(
    name = "navsim_gmapping",
    data = [
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "sight",
    ],
    script = "select_json_gmapping.sh",
    visibility = ["//visibility:public"],
)
