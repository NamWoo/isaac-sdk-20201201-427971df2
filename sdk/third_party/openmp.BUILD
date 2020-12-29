"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "openmp",
    srcs = [
    ] + select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": [
            "gcc/x86_64-linux-gnu/7/libgomp.a",
        ],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": [
            "gcc-cross/aarch64-linux-gnu/7/libgomp.a",
        ],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "openmp_dynamic",
    srcs = [
    ] + select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": [
            "gcc/x86_64-linux-gnu/7/libgomp.so",
        ],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": [
            "gcc-cross/aarch64-linux-gnu/7/libgomp.so",
        ],
    }),
    visibility = ["//visibility:public"],
)
