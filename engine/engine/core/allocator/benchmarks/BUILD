"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary")

isaac_cc_binary(
    name = "allocator",
    srcs = ["allocator.cpp"],
    deps = [
        "//engine/core",
        "//engine/core/allocator",
        "//engine/core/allocator:test_utils",
        "@benchmark",
    ],
)
