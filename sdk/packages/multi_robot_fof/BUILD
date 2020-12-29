"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:py.bzl", "isaac_py_app")

isaac_py_app(
    name = "build_graph",
    srcs = [
        "build_graph.py",
    ],
    data = [
        "pose2_graph_builder.config.json",
        "//packages/multi_robot_fof/assets:maps",
        "//packages/path_planner/apps:pose2_graph_builder",
    ],
    modules = [
        "map",
        "path_planner",
        "planner",
        "sight",
    ],
)

isaac_py_app(
    name = "transporter",
    srcs = [
        "transporter.py",
    ],
    data = [
        "pose2_planner.config.json",
        "transporter.app.json",
        "//packages/behavior_tree/apps:missions",
        "//packages/cart_delivery/apps:navigate",
        "//packages/multi_robot_fof/assets:maps",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "map",
        "sight",
    ],
)

isaac_py_app(
    name = "station",
    srcs = [
        "station.py",
    ],
    data = [
        "station.app.json",
        "ur10.config.json",
        "//apps/assets/kinematic_trees",
        "//apps/samples/pick_and_place",
        "//packages/behavior_tree/apps:missions",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
    ],
    modules = [
        "sight",
    ],
)

filegroup(
    name = "py_init",
    data = ["__init__.py"],
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "station_mission",
    data = [
        "scenario_manager.py",
        "//packages/missions",
    ],
    modules = [
        "engine_tcp_udp",
        "sight",
    ],
)

isaac_py_app(
    name = "transporter_mission",
    data = [
        "pose2_planner.config.json",
        "scenario_manager.py",
        "//packages/missions",
        "//packages/multi_robot_fof/assets:maps",
    ],
    modules = [
        "engine_tcp_udp",
        "map",
        "navigation",
        "sight",
        "planner",
        "utils",
        "viewers",
    ],
)

isaac_py_app(
    name = "factory_mission",
    data = [
        "scenario_manager.py",
        "station_mission",
        "transporter_mission",
    ],
    modules = [
        "engine_tcp_udp",
        "map",
        "navigation",
        "sight",
    ],
)
