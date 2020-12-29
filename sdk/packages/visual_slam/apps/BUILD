"""
Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_graph")
load("//bzl:py.bzl", "isaac_py_app")

isaac_py_app(
    name = "sim_svio",
    srcs = [
        "sim_svio.py",
    ],
    data = [
        "stereo_visual_odometry_rgb_subgraph",
        "//apps/assets/maps",
        "//packages:py_init",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "atlas",
        "behavior_tree",
        "map",
        "navsim",
        "sight",
        "utils",
        "viewers",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_py_app(
    name = "sim_svio_joystick",
    srcs = [
        "sim_svio_joystick.py",
    ],
    data = [
        "stereo_visual_odometry_rgb_subgraph",
        "//apps/assets/maps",
        "//packages:py_init",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_imu_odometry_subgraph",
        "//packages/navsim/apps:navsim_navigation_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "atlas",
        "behavior_tree",
        "map",
        "navsim",
        "sight",
        "utils",
        "viewers",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_app(
    name = "stereo_vo",
    app_json_file = "stereo_vo.app.json",
    data = [
        "stereo_visual_odometry_grayscale_subgraph",
    ],
    modules = [
        "sight",
        "visual_slam",
        "utils",
        "zed",
    ],
)

isaac_py_app(
    name = "svo_zed",
    srcs = [
        "svo_zed.py",
    ],
    args = ["--no-imu"],
    data = [
        "stereo_visual_odometry_grayscale_subgraph",
        "//packages:py_init",
    ],
    modules = [
        "sight",
        "zed",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_py_app(
    name = "svo_zed_imu",
    srcs = [
        "svo_zed.py",
    ],
    args = ["--imu"],
    data = [
        "stereo_visual_odometry_grayscale_subgraph",
        "//packages:py_init",
    ],
    main = "svo_zed.py",
    modules = [
        "sight",
        "viewers",
        "perception",
        "zed",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_py_app(
    name = "svo_realsense",
    srcs = [
        "svo_realsense.py",
    ],
    data = [
        "stereo_visual_odometry_grayscale_subgraph",
        "//packages:py_init",
    ],
    modules = [
        "sight",
        "visual_slam",
        "realsense",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_graph(
    name = "stereo_visual_odometry_rgb_subgraph",
    graph = "stereo_visual_odometry_rgb.subgraph.json",
    data = [
        "stereo_visual_odometry_grayscale_subgraph",
    ],
    modules = [
        "visual_slam",
        "perception",
    ],
    visibility = ["//visibility:public"],
)

isaac_graph(
    name = "stereo_visual_odometry_grayscale_subgraph",
    graph = "stereo_visual_odometry_grayscale.subgraph.json",
    modules = [
        "behavior_tree",
        "visual_slam",
        "sight",
        "utils",
        "viewers",
    ],
    visibility = ["//visibility:public"],
)
