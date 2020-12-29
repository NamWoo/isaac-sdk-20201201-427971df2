"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_subgraph")
load("//bzl:py.bzl", "isaac_py_app")

filegroup(
    name = "config",
    srcs = [
        "detection_pose_estimation_cnn_inference.config.json",
        "detection_pose_estimation_cnn_inference_dolly.config.json",
        "detection_pose_estimation_cnn_inference_kltSmall.config.json",
    ],
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "simulation_subgraph",
    modules = [
        "engine_tcp_udp",
        "ml",
        "navsim",
        "viewers",
    ],
    subgraph = "pose_estimation_sim.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "pose_estimation_cnn_subgraph",
    data = [
    ],
    modules = [
        "ml",
        "object_pose_estimation",
        "utils",
    ],
    subgraph = "pose_estimation_cnn.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "detection_pose_estimation_cnn_inference_app",
    srcs = ["detection_pose_estimation_cnn_inference_app.py"],
    data = [
        "config",
        "detection_pose_estimation_cnn_inference",
        "simulation_subgraph",
        "//apps:py_init",
        "//packages/cask/apps:replay_subgraph",
        "@industrial_dolly_pose_estimation_data",
        "@sortbot_pose_estimation_data",
        "@industrial_dolly_sortbot_sample_images",
    ],
    modules = [
        "message_generators",
        "ml",
        "object_pose_estimation",
        "realsense",
        "sensors:v4l2_camera",
        "sight",
        "viewers",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_py_app(
    name = "detection_pose_estimation_cnn_inference_deploy_app",
    srcs = ["detection_pose_estimation_cnn_inference_deploy_app.py"],
    data = [
        "config",
        "detection_pose_estimation_cnn_inference",
        "detection_pose_estimation_cnn_inference_app.py",
        "simulation_subgraph",
        "//apps:py_init",
        "//packages/cask/apps:replay_subgraph",
        "@industrial_dolly_sortbot_sample_images",
    ],
    modules = [
        "message_generators",
        "ml",
        "object_pose_estimation",
        "realsense",
        "sensors:v4l2_camera",
        "sight",
        "viewers",
    ],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_py_app(
    name = "detection_pose_estimation_cnn_inference_ros_bridge_app",
    srcs = ["detection_pose_estimation_cnn_inference_ros_bridge_app.py"],
    data = [
        "config",
        "detection_pose_estimation_cnn_inference",
        "//apps:py_init",
        "//packages/ros_bridge/apps:ros_to_perception_subgraph",
        "@industrial_dolly_pose_estimation_data",
        "@sortbot_pose_estimation_data",
    ],
    modules = [
        "ml",
        "object_pose_estimation",
        "sight",
        "viewers",
    ],
    tags = ["manual"],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_subgraph(
    name = "detection_pose_estimation_cnn_inference",
    data = [
        "config",
        "pose_estimation_cnn_subgraph",
        "//packages/detect_net/apps:detect_net_inference",
        "@industrial_dolly_pose_estimation_cnn_model",
        "@sortbot_pose_estimation_models",
    ],
    modules = [
        "utils",
        "sight",
        "viewers",
    ],
    subgraph = "detection_pose_estimation_cnn_inference.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "pose_estimation_inference_sim_groundtruth_detection",
    app_json_file = "pose_estimation_inference_sim_groundtruth_detection.app.json",
    data = [
        "pose_estimation_cnn_subgraph",
        "simulation_subgraph",
        "@industrial_dolly_pose_estimation_cnn_model",
        "@sortbot_pose_estimation_models",
    ],
    modules = [
        "json",
        "ml",
        "object_pose_estimation",
        "sight",
        "viewers",
    ],
)
