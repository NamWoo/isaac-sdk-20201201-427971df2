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
    name = "generate_kitti_dataset",
    data = [
        "//packages/navsim/apps:navsim_training_subgraph",
    ],
    modules = [
        "ml",
        "perception",
        "sight",
        "viewers",
    ],
)

isaac_py_app(
    name = "verify_kitti_dataset",
    srcs = ["verify_kitti_dataset.py"],
    tags = ["manual"],
    deps = [
        "//packages/pyalice",
    ],
)
