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
    name = "localization_subgraph",
    modules = [
        "atlas",
        "behavior_tree",
        "flatscan_localization",
        "monitor",
        "navigation",
        "utils",
        "viewers",
    ],
    subgraph = "localization.subgraph.json",
    visibility = ["//visibility:public"],
)
