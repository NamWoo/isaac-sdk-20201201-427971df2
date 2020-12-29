"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "obstacles",
    hdrs = [
        "distance_map_obstacle.hpp",
        "obstacle.hpp",
        "obstacle_with_pose2.hpp",
        "polygon_obstacle.hpp",
        "spherical_obstacle.hpp",
    ],
    srcs = [
        "distance_map_obstacle.cpp",
        "polygon_obstacle.cpp",
        "spherical_obstacle.cpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/geometry:line_utils",
        "@com_nvidia_isaac_engine//engine/gems/geometry:types",
        "@com_nvidia_isaac_engine//engine/gems/interpolation",
        "@com_nvidia_isaac_engine//engine/gems/image",
    ],
)
