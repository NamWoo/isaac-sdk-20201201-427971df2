"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//doc:doc.bzl", "isaac_doc_dep")

isaac_doc_dep(
    name = "manipulation",
    srcs = [
        "images/kinematic_chain_example.jpg",
        "images/motion_planning.jpg",
        "images/shuffle_box_detection.jpg",
        "images/shuffle_box_perspective.jpg",
        "images/simple_joint_control.png",
        "index.rst",
        "kinematics.rst",
        "motion_planning.rst",
        "samples.rst",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//apps/samples/pick_and_place/doc",
    ],
)
