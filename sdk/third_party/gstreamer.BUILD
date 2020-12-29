"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# GStreamer pipeline-based multimedia framework with various media processing systems.
# Support for GStreamer functionality and application interfaces included.
cc_library(
    name = "gstreamer",
    srcs = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": glob([
            "x86_64/lib/*.so",
        ]),
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": glob([
            "aarch64_jetpack44/lib/*.so",
        ]),
    }),
    hdrs = glob([
        "include/**/*.h",
    ]),
    includes = [
        "include",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = [
        "@glib",
    ],
)
