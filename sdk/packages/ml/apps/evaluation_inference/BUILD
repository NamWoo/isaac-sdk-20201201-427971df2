"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:py.bzl", "isaac_py_app")

isaac_py_app(
    name = "evaluation_inference",
    srcs = ["evaluation_inference.py"],
    data = [
        "//packages/cask/apps:multi_cask_processing",
        "//packages/cask/apps:record_subgraph",
        "//packages/cask/apps:replay_subgraph",
        "//packages/detect_net/evaluation:detect_net_eval_inference",
        "//packages/object_pose_estimation/apps/pose_cnn_decoder/evaluation:pose_cnn_decoder_inference_record",
    ],
    modules = [
    ],
    deps = [
        "//packages/pyalice",
    ],
)
