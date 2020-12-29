"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_component")

filegroup(
    name = "libplanner_module.so",
    tags = ["manual"],
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64":
        ["//packages_x86_64/planner:libplanner_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/planner:libplanner_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "robot_shape_component",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/map:obstacle_atlas",
        "//packages/navigation/gems:robot_shape",
    ],
)

isaac_component(
    name = "spherical_robot_shape_component",
    visibility = ["//visibility:public"],
    deps = [
        ":robot_shape_component",
        "//packages/navigation/gems:spherical_robot_shape",
    ],
)
