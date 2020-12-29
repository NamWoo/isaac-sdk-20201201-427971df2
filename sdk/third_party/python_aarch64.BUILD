"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# AARCH64 Python3.6 binary and headers pack grabbed as-is from Jetpack 4.2
cc_library(
    name = "python_aarch64",
    srcs = [
        "lib/config-aarch64-linux-gnu/libpython3.6.so",
    ],
    hdrs = glob([
        "include/python3.6/**/*.h",
    ]),
    linkopts = [],
    strip_include_prefix = "include//python3.6",
    visibility = ["//visibility:public"],
    deps = [
        ":python_aarch64_hdr",
    ],
)

cc_library(
    name = "python_aarch64_hdr",
    srcs = [],
    hdrs = glob([
        "include/aarch64-linux-gnu/python3.6m/*.h",
    ]),
    linkopts = [],
    strip_include_prefix = "include/",
    deps = [],
)
