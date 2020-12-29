"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary")
load("//bzl:module.bzl", "isaac_pkg")

isaac_cc_binary(
    name = "aliasing",
    srcs = ["aliasing.cpp"],
    deps = [
        "@benchmark",
    ],
)

isaac_cc_binary(
    name = "compression",
    srcs = ["benchmark.cpp"],
    data = ["@com_nvidia_isaac_engine//engine/gems/image/data"],
    deps = [
        "@benchmark",
        "@com_nvidia_isaac_engine//engine/gems/image",
    ],
)

isaac_cc_binary(
    name = "normalize",
    srcs = ["normalize.cpp"],
    data = ["@com_nvidia_isaac_engine//engine/gems/image/data"],
    deps = [
        "@benchmark",
        "@com_nvidia_isaac_engine//engine/gems/image",
        "@com_nvidia_isaac_engine//engine/gems/image:cuda_conversions",
    ],
)

isaac_pkg(
    name = "image_benchmark-pkg",
    srcs = [
        "aliasing",
        "compression",
        "normalize",
    ],
)
