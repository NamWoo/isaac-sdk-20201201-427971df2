"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:py.bzl", "isaac_py_app")
load("//bzl:module.bzl", "isaac_subgraph")

isaac_subgraph(
    name = "dope_inference_subgraph",
    modules = [
        "ml",
        "object_pose_estimation",
        "perception",
        "viewers",
    ],
    subgraph = "dope_inference.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "dope_inference",
    srcs = ["dope_inference.py"],
    data = [
        "dope_inference_subgraph",
        "//packages/cask/apps:replay_subgraph",
        "//packages/navsim/apps:navsim_tcp_subgraph",
        "@dope_ycb_data",
    ],
    modules = [
        "message_generators",
        "realsense",
        "sight",
        "utils",
    ],
)

py_binary(
    name = "dope_model",
    srcs = ["dope_model.py"],
    main = "dope_model.py",
)

