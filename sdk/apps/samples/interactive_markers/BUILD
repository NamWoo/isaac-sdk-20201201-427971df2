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
    name = "virtual_arm",
    srcs = ["VirtualArm.cpp"],
    hdrs = ["VirtualArm.hpp"],
    visibility = ["//visibility:public"],
    deps = [],
)

isaac_app(
    name = "interactive_markers",
    data = ["//apps/assets/maps"],
    modules = [
        "flatsim",
        "sight",
        "//apps/samples/interactive_markers:virtual_arm",
    ],
)
