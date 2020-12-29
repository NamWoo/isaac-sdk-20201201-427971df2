"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# This package contains the aarch64 version of the libi2c library copied from xavier
cc_library(
    name = "libi2c_aarch64",
    srcs = [
        "lib_aarch64/libi2c.so",
    ],
    hdrs = [
        "include/i2c/smbus.h",
    ],
    includes = ["include"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
