"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")
cc_import(
    name = "liblaikago_comm",
    shared_library = "lib/liblaikago_comm.so",
)

cc_library(
    name = "laikago_sdk",
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        ":liblaikago_comm",
        "@lcm//:lcm",
    ],
)
