"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app")

# Application to generate a map from previously recorded logs. The application uses the
# GMapping tool. Logs file path can be supplied via Replay Control Panel in Sight. You
# will need a full scan and an odometry (or pose) logs.
isaac_app(
    name = "log_gmapping",
    data = [
        "//packages/cask/apps:replay_subgraph",
        "//packages/lidar_slam/apps:differential_base_gmapping_subgraph",
    ],
    modules = [
        "perception",
    ],
)
