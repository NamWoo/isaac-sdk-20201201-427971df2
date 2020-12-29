"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_component")

isaac_component(
    name = "dynamixel_driver",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/dynamixel/gems:dynamixel",
        "//packages/engine_gems/state:io",
        "@com_nvidia_isaac_engine//engine/core/tensor:universal_tensor",
        "@com_nvidia_isaac_engine//engine/gems/tensor",
    ],
)
