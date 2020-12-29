"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app")

isaac_app(
    name = "carter_record",
    app_json_file = "carter_record.app.json",
    data = [
        "//apps/carter:2d_carter_subgraph",
        "//apps/carter/robots",
        "//packages/cask/apps:record_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
    ],
    modules = [
        "sight",
        "viewers",
        "zed",
    ],
)
