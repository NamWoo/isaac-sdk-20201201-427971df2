"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "multi_joint_lqr_control_subgraph",
    modules = [
        "controller",
        "lqr",
        "map",
        "planner",
    ],
    subgraph = "multi_joint_lqr_control.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "multi_joint_rmp_control_subgraph",
    modules = [
        "controller",
        "map",
        "planner",
        "rmpflow",
    ],
    subgraph = "multi_joint_rmp_control.subgraph.json",
    visibility = ["//visibility:public"],
)
