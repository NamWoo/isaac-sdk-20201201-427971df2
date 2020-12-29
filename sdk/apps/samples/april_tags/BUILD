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

isaac_app(
    name = "april_tags",
    modules = [
        "//packages/fiducials:april_tags",
        "sensors:v4l2_camera",
        "sight",
        "viewers",
    ],
)

isaac_py_app(
    name = "april_tags_python",
    srcs = [
        "april_tags_python.py",
    ],
    data = [
        "april_tags_python.config.json",
        "//packages:py_init",
    ],
    modules = [
        "//packages/fiducials:april_tags",
        "realsense",
        "sensors:v4l2_camera",
        "sight",
        "viewers",
        "zed",
    ],
)

isaac_py_app(
    name = "nvapril_tag_vs_april_tag3",
    srcs = [
        "nvapril_tag_vs_april_tag3.py",
    ],
    data = ["//packages/fiducials/assets",
            "//packages/navsim/apps:navsim_tcp_subgraph"],
    modules = [
        "//packages/fiducials:april_tags",
        "//packages/fiducials:april_tags3",
        "sensors:v4l2_camera",
        "viewers",
        "message_generators",
        "sight"
    ],
    deps = [
        "//packages/pyalice",
    ],
)
