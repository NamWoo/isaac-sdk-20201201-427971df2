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
    name = "carter_delivery_codelet",
    srcs = ["CarterDelivery.cpp"],
    hdrs = ["CarterDelivery.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/behavior_tree/components",
        "//packages/map:public_headers",
        "@com_nvidia_isaac_engine//engine/gems/algorithm:string_utils",
        "@com_nvidia_isaac_engine//engine/gems/state_machine",
    ],
)

isaac_app(
    name = "carter_delivery",
    data = [
        "//apps/assets/maps",
        "//apps/carter:2d_carter_subgraph",
        "//apps/carter/carter_delivery/operators",
        "//apps/carter/robots",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
    ],
    modules = [
        "//apps/carter/carter_delivery:carter_delivery_codelet",
        "behavior_tree",
        "communications",
        "map",
        "navigation",
        "perception",
        "planner",
        "segway",
        "sight",
        "imu",
        "sensors:joystick",
        "velodyne_lidar",
        "viewers",
    ],
)

isaac_app(
    name = "carter_delivery_flatsim",
    data = [
        "//apps/assets/maps",
        "//apps/carter/carter_delivery/operators",
        "//packages/flatsim/apps:2d_differential_base_simulation_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
    ],
    modules = [
        "//apps/carter/carter_delivery:carter_delivery_codelet",
        "flatsim",
        "communications",
        "map",
        "navigation",
        "perception",
        "planner",
        "sight",
        "imu",
        "viewers",
    ],
)
