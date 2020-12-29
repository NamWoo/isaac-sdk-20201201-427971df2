"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "python",
    srcs = [
        "lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6.so",
    ],
    hdrs = glob([
        "include/python3.6/**/*.h",
    ]),
    linkopts = [],
    strip_include_prefix = "include/python3.6",
    visibility = ["//visibility:public"],
    deps = [],
)
