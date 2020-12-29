"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_pkg", "isaac_subgraph")

py_binary(
    name = "pose_estimation_cnn_training",
    srcs = [
        "__init__.py",
        "pose_estimation_cnn_training.py",
    ],
    data = [
        "training.app.json",
        "training_config.json",
        "decoder.py",
        "encoder.py",
        "encoder_bbox.py",
        "pose_estimation_cnn.py",
        "pose_estimation_cnn_utils.py",
        "regression_rotation.py",
        "regression_translation.py",
        "//packages/ml:libml_module.so",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/evaluation:pose_evaluation_utils",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/training:data_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/training:simulation_subgraph",
        "//packages/engine_tcp_udp:libengine_tcp_udp_module.so",
        "//packages/sight:libsight_module.so",
        "//packages/viewers:libviewers_module.so",
    ],
    deps = [
        "//packages/ml:pyml",
        "//packages/pyalice",
    ],
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
    name = "data_subgraph",
    data = [
        ":simulation_subgraph",
    ],
    modules = [
        "ml",
        "object_pose_estimation",
        "utils",
    ],
    subgraph = "training_data_provider.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_pkg(
    name = "pose_estimation_cnn_training-pkg",
    srcs = [
        ":pose_estimation_cnn_training",
    ],
    data = [
        "//packages:py_init",
        "//packages/object_pose_estimation:py_init",
    ],
)

isaac_app(
    name = "generate_training_validation_data",
    app_json_file = "generate_training_validation_data.app.json",
    data = [
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/training:data_subgraph",
    ],
    modules = [
        "json",
        "ml",
        "object_pose_estimation",
        "sight",
        "viewers",
    ],
)

isaac_app(
    name = "training",
    app_json_file = "training.app.json",
    data = [
        ":data_subgraph",
    ],
    modules = [
        "json",
        "ml",
        "object_pose_estimation",
        "sight",
        "viewers",
    ],
)