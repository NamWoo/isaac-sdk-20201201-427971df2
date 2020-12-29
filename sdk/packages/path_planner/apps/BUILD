"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_graph", "isaac_subgraph")

isaac_app(
    name = "pose2_graph_builder",
    data = [
        "pose2_graph_builder_subgraph",
        "//apps/assets/maps",
    ],
    modules = [
        "map",
        "path_planner",
        "planner",
        "sight",
    ],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "pose2_graph_builder_subgraph",
    graph = "pose2_graph_builder.subgraph.json",
    modules = [
        "path_planner",
    ],
    visibility = ["//visibility:public"],
)
