
"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
load("//bzl:module.bzl", "isaac_cc_module", "isaac_component")

filegroup(
    name = "libmap_module.so",
    tags = ["manual"],
    data = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/map:libmap_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/map:libmap_module.so_release"],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "public_headers",
    hdrs = [
        "Layer.hpp",
        "PolygonMapLayer.hpp",
        "WaypointMapLayer.hpp",
    ],
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["//packages_x86_64/map:libmap_module.so_release"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["//packages_jetpack44/map:libmap_module.so_release"],
    }),
    deps = [":obstacle_atlas"],
    visibility = ["//visibility:public"],
)

isaac_cc_module(
    name = "kinematic",
    deps = [
        "kinematic_tree",
    ],
)

isaac_component(
    name = "kinematic_tree",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/math/gems/kinematic_tree",
        "//packages/math/gems/kinematic_tree:json_loader",
        "@com_nvidia_isaac_engine//engine/core/math:dual_quaternion",
    ],
)

isaac_component(
    name = "obstacle_atlas",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/image",
        "//packages/map/gems:obstacles",
    ],
)

isaac_component(
    name = "flatmap_cost",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/navigation/gems:robot_shape",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_component(
    name = "flatmap_cost_combination",
    visibility = ["//visibility:public"],
    deps = [
        ":flatmap_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)
