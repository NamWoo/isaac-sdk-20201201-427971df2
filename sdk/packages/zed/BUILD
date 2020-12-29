"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_cc_module", "isaac_component")

isaac_component(
    name = "zed_camera",
    linkopts = [
        "-ldl",
        "-lgomp",
        "-lrt",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/zed/gems:time_offset_calculator",
        "//third_party:cuda",
        "//third_party:zed",
        "@com_nvidia_isaac_engine//engine/gems/geometry:pinhole",
        "@com_nvidia_isaac_engine//engine/gems/system:cuda_context",
    ],
)

isaac_cc_module(
    name = "zed",
    deps = [
        ":zed_camera",
        "//packages/zed/components:zed_imu_reader",
    ],
)
