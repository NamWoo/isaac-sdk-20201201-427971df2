"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "elbrus_vo",
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["lib_x86_64/libelbrus.so"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["lib_aarch64_xavier/libelbrus.so"],
    }),
    hdrs = [
        "include/elbrus.h",
    ],
    includes = ["include"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
