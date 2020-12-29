"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "differential_base",
    srcs = ["differential_base.cpp"],
    hdrs = ["differential_base.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/composite/gems:parser",
    ],
)

isaac_cc_library(
    name = "ackermann_base",
    srcs = ["ackermann_base.cpp"],
    hdrs = ["ackermann_base.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/composite/gems:parser",
    ],
)
