"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "isaac_ros_bridge_aarch64_xavier",
    srcs = glob(
        ["lib/*.so*"],
    ),
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
        "include/**/*.ipp",
    ]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
