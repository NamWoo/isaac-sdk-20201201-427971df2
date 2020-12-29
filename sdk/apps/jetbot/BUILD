"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
load("//bzl:module.bzl", "isaac_subgraph")
load("//bzl:py.bzl", "isaac_jupyter_app")

isaac_jupyter_app(
    name = "jetbot_jupyter_notebook",
    data = [
        ":simulation_tcp_subgraph",
        ":virtual_gamepad_subgraph",
        ":v4l2_camera_subgraph",
        "//packages/detect_net/apps:detect_net_inference",
        "@jetbot_ball_detection_resnet_model",
    ],
    modules = [
        "message_generators",
        "navigation",
        "perception",
        "sensors:v4l2_camera",
        "sight",
        "viewers",
    ],
    notebook = "jetbot_notebook.ipynb",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "simulation_tcp_subgraph",
 modules = [
        "engine_tcp_udp",
    ],
    subgraph = "simulation_tcp.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "virtual_gamepad_subgraph",
    modules = [
        "navigation",
        "sight"
    ],
    subgraph = "virtual_gamepad.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "v4l2_camera_subgraph",
    modules = [
        "sensors:v4l2_camera",
        "viewers",
    ],
    subgraph = "v4l2_camera.subgraph.json",
    visibility = ["//visibility:public"],
)

