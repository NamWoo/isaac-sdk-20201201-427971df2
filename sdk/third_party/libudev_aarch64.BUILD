"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# This package contains the aarch64 version of the libudev library copied from tx2
# libudev version 229-4ubuntu21.4
cc_library(
    name = "libudev_aarch64",
    srcs = [
        "usr/lib/aarch64-linux-gnu/libudev.so",
    ],
    hdrs = [
        "usr/include/libudev.h",
    ],
    includes = [
        "usr/include",
    ],
    visibility = ["//visibility:public"],
)
