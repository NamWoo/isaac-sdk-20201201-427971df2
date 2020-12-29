"""
Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_graph", "isaac_pkg")
load("//bzl:py.bzl", "isaac_py_app")

isaac_graph(name = "demo_1", modules = ["map", "navigation"], data = ["//apps/assets/maps"])
isaac_graph(name = "demo_2", modules = ["map", "navigation"], data = ["//apps/assets/maps"])
isaac_graph(name = "demo_3", modules = ["map", "navigation"], data = ["//apps/assets/maps"])
isaac_graph(name = "demo_4", modules = ["atlas", "flatsim", "map", "message_generators",
                                        "navigation"],
            data = ["//apps/assets/maps"])
isaac_graph(name = "demo_5", modules = ["atlas", "flatsim", "map", "message_generators",
                                        "navigation", "planner_cost"],
            data = ["//apps/assets/maps"])
isaac_graph(name = "demo_6", modules = ["atlas", "flatsim", "map", "message_generators",
                                        "navigation", "planner_cost"],
            data = ["//apps/assets/maps"])
isaac_graph(name = "demo_7", modules = ["atlas", "flatsim", "map", "message_generators",
                                        "navigation", "planner_cost", "utils"],
            data = ["//apps/assets/maps",
                    "//packages/navigation/apps:differential_base_commander_subgraph",
                    "//packages/otg5/apps:otg5_straight_differential_base_subgraph"])

filegroup(
    name = "maps",
    data = [
        "demo_1",
        "demo_2",
        "demo_3",
        "demo_4",
        "demo_5",
        "demo_6",
        "demo_7",
    ],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "carter_config",
    graph = "carter.config.json",
)

isaac_graph(
    name = "str_config",
    graph = "str.config.json",
)

isaac_graph(
    name = "str_dolly_config",
    graph = "str_dolly.config.json",
)

filegroup(
    name = "robot_configs",
    data = [
        "carter_config",
        "str_config",
        "str_dolly_config",
    ],
)

isaac_py_app(
    name = "flatsim",
    srcs = [
        "__init__.py",
        "flatsim.py",
    ],
    data = [
        "flatsim_subgraph",
        "maps",
        "pose2_graph_planner_config",
        "robot_configs",
        "//packages:py_init",
        "//packages/behavior_tree/apps:missions",
        "//packages/flatsim:py_init",
        "//packages/path_planner/apps:pose2_graph_builder_subgraph",
    ],
    modules = [
        "flatsim",
    ],
)

isaac_graph(
    name = "2d_differential_base_simulation_subgraph",
    graph = "2d_differential_base_simulation.subgraph.json",
    modules = [
        "//packages/atlas",
        "//packages/flatsim",
        "//packages/message_generators"
    ],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "pose2_graph_planner_config",
    graph = "pose2_graph_planner.config.json",
    modules = ["//packages/flatsim"],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "flatsim_subgraph",
    data = [
        "2d_differential_base_simulation_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
    ],
    graph = "flatsim.subgraph.json",
    visibility = ["//visibility:public"],
)
