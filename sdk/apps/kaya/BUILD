"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_subgraph")

isaac_app(
    name = "joystick",
    data = [
        ":2d_kaya_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
    ],
    modules = [
        "sight",
    ],
)

isaac_app(
    name = "navigate",
    data = [
        ":2d_kaya_subgraph",
        "//apps/assets/maps",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_navigation_subgraph",
    ],
    modules = [
        "map",
        "sight",
    ],
)

isaac_app(
    name = "object_detection",
    data = [
        ":2d_kaya_subgraph",
        "//packages/detect_net/apps:detect_net_inference",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
        "@tennis_ball_resnet_object_detection_model",
    ],
    modules = [
        "ml",
        "navigation",
        "sight",
        "viewers",
    ],
)

isaac_app(
    name = "follow_me",
    data = [
        ":kaya_hardware_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_control_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
    ],
    modules = [
        "fiducials",
        "fiducials:april_tags",
        "navigation",
        "planner",
        "sight",
        "viewers",
    ],
)

isaac_app(
    name = "gmapping_distributed_host",
    modules = [
        "engine_tcp_udp",
        "lidar_slam:g_mapping",
        "sight",
    ],
)

isaac_app(
    name = "gmapping_distributed_kaya",
    data = [
        ":2d_kaya_subgraph",
        "//packages/egm_fusion:evidence_grid_map_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
    ],
    modules = [
        "engine_tcp_udp",
        "navigation",
        "perception",
        "planner",
        "sight",
        "viewers",
    ],
)

isaac_subgraph(
    name = "kaya_hardware_subgraph",
    modules = [
        "dynamixel",
        "imu",
        "kaya",
        "realsense",
        "viewers",
    ],
    subgraph = "kaya_hardware.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "2d_kaya_subgraph",
    data = [
        ":kaya_hardware_subgraph",
    ],
    modules = [
        "rgbd_processing",
    ],
    subgraph = "2d_kaya.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "joystick_sim",
    data = [
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
    ],
    modules = [
        "sight",
    ],
)
