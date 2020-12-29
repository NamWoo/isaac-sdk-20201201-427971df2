"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_graph", "isaac_subgraph")
load("//bzl:py.bzl", "isaac_py_app")

isaac_graph(
    name = "pick_task",
    modules = [
        "atlas",
        "behavior_tree",
        "composite",
        "path_planner",
        "utils",
    ],
)

isaac_graph(
    name = "place_task",
    modules = [
        "behavior_tree",
        "composite",
        "path_planner",
        "utils",
    ],
)

isaac_subgraph(
    name = "block_pose_estimation_subgraph",
    data = [
        "//packages/object_pose_refinement/apps:pose_refinement_subgraph",
        "//packages/superpixels/apps:superpixels_subgraph",
        "@block_pose_estimation_data",
    ],
    modules = [
        "sight",
        "surflets",
        "viewers",
    ],
    subgraph = "block_pose_estimation.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "block_pose_estimation",
    srcs = [
        "block_pose_estimation.py",
    ],
    data = [
        "__init__.py",
        "block_pose_estimation_subgraph",
        "//apps:py_init",
        "//apps/samples:py_init",
        "//packages/cask/apps:replay_subgraph",
        "//packages/navsim/apps:navsim_tcp_subgraph",
    ],
    modules = [
        "realsense",
        "sight",
        "utils",
    ],
)

isaac_py_app(
    name = "pick_and_place",
    srcs = [
        "pick_and_place.py",
    ],
    data = [
        "block_pose_estimation",
        "franka.config.json",
        "pick_and_place.app.json",
        "pick_task",
        "place_task",
        "py_init",
        "smallKLT_detection_pose_estimation.config.json",
        "task_planner.py",
        "ur10.config.json",
        "//apps:py_init",
        "//apps/assets/kinematic_trees",
        "//apps/samples:py_init",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "//packages/planner/apps:multi_joint_lqr_control_subgraph",
    ],
    modules = [
        "behavior_tree",
        "composite",
        "object_pose_estimation",
        "sight",
        "utils",
        "viewers",
    ],
)

filegroup(
    name = "py_init",
    data = ["__init__.py"],
    visibility = ["//visibility:public"],
)
