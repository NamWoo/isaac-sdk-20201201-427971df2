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
    name = "sight_image_test",
    srcs = ["sight_image_test.cpp"],
    data = [
        "@com_nvidia_isaac_engine//engine/gems/image/data",
        "@isaac_assets",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/alice",
        "@com_nvidia_isaac_engine//engine/alice/tools:parse_command_line",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/gems/image",
        "//messages",
        "//packages/sight:libsight_module.so",
    ],
)
