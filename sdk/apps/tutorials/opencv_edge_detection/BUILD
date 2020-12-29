"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "edge_detector",
    srcs = [
        "EdgeDetector.cpp",
    ],
    hdrs = [
        "EdgeDetector.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
        "@com_nvidia_isaac_engine//engine/core/image",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/gems/sight",
    ],
)

isaac_app(
    name = "opencv_edge_detection",
    modules = [
        "//apps/tutorials/opencv_edge_detection:edge_detector",
        "sensors:v4l2_camera",
        "sight",
        "viewers",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "opencv_unity3d",
    data = [
        "//packages/navsim/apps:navsim_navigation_subgraph",
    ],
    modules = [
        "//apps/tutorials/opencv_edge_detection:edge_detector",
        "sight",
        "viewers",
    ],
    visibility = ["//visibility:public"],
)
