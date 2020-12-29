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
    name = "otg5_straight_differential_base_subgraph",
    modules = [
        "behavior_tree",
        "otg5",
    ],
    subgraph = "otg5_straight_differential_base.subgraph.json",
    visibility = ["//visibility:public"],
)
