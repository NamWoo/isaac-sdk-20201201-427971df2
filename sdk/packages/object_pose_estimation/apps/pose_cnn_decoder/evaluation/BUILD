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
        "evaluation_config.json",
    ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "pose_evaluation_utils",
    srcs = [
        "pose_evaluation_utils.py",
    ],
    data = [
        "cuboid.py",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "pose_cnn_decoder_inference_record",
    app_json_file = "pose_cnn_decoder_inference_record.app.json",
    data = [
        "//packages/cask/apps:record_subgraph",
        "//packages/cask/apps:replay_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:config",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder:detection_pose_estimation_cnn_inference",
        "@industrial_dolly_pose_evaluation_data",
    ],
    modules = [
        "json",
    ],
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "pose_cask_evaluation",
    srcs = [
        "pose_cask_evaluation.py",
    ],
    data = [
        "config",
        "//packages/cask/apps:multi_cask_processing",
        "//packages/json:libjson_module.so",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/evaluation:pose_evaluation_utils",
        "@industrial_dolly_pose_evaluation_data",
    ],
)

isaac_subgraph(
    name = "gt_pose_from_april_tags",
    data = [
    ],
    modules = [
        "fiducials:april_tags",
        "object_pose_estimation",
        "sight",
        "utils",
        "viewers",
    ],
    subgraph = "gt_pose_from_april_tags.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "record_groundtruth_pose",
    app_json_file = "record_groundtruth_pose.app.json",
    data = [
        "//packages/cask/apps:record_subgraph",
        "//packages/cask/apps:replay_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/evaluation:gt_pose_from_april_tags",
    ],
    modules = [
        "json",
    ],
    visibility = ["//visibility:public"],
)

isaac_py_app(
    name = "record_multicask_groundtruth_pose",
    srcs = ["record_multicask_groundtruth_pose.py"],
    data = [
        "//packages/cask/apps:multi_cask_processing",
        "//packages/cask/apps:record_subgraph",
        "//packages/cask/apps:replay_subgraph",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/evaluation:record_groundtruth_pose",
    ],
    modules = [
    ],
    deps = [
        "//packages/pyalice",
    ],
)
