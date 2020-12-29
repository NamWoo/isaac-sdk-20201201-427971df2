"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")
cc_library(
    name = "lcm",
    srcs = glob(
        [
            "lcm/**/*.c",
            "lcm/**/*.cpp",
        ],
        exclude = [
            "lcm/windows/*.cpp",
        ],
    ),
    hdrs = glob([
        "lcm/**/*.h",
        "lcm/**/*.hpp",
    ]),
    copts = [
        "-Wno-deprecated-declarations",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@glib",
    ],
)
