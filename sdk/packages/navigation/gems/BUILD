"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "robot_shape",
    hdrs = [
        "robot_shape.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = ["//packages/map/gems:obstacles"],
)

isaac_cc_library(
    name = "spherical_robot_shape",
    srcs = [
        "spherical_robot_shape.cpp",
    ],
    hdrs = [
        "spherical_robot_shape.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":robot_shape",
        "//packages/map/gems:obstacles",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)
