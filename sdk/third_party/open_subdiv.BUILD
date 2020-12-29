"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

exports_files(["LICENSE.txt"])

cc_library(
    name = "far",
    srcs = glob([
        "opensubdiv/far/**/*.cpp",
        "opensubdiv/far/**/*.c",
    ]) + ["opensubdiv/version.h"],
    hdrs = glob([
        "opensubdiv/far/**/*.hpp",
        "opensubdiv/far/**/*.h",
    ]),
    copts = [
        "-Wno-strict-aliasing",
        "-Wno-unused-but-set-variable",
        "-Wno-unused-function",
    ],
    defines = [],
    visibility = ["//visibility:public"],
    deps = [":vtr"],
)

cc_library(
    name = "vtr",
    srcs = glob([
        "opensubdiv/vtr/**/*.cpp",
        "opensubdiv/vtr/**/*.c",
    ]) + ["opensubdiv/version.h"],
    hdrs = glob([
        "opensubdiv/vtr/**/*.hpp",
        "opensubdiv/vtr/**/*.h",
    ]),
    copts = ["-Wno-strict-aliasing"],
    defines = [],
    visibility = ["//visibility:public"],
    deps = [":sdc"],
)

cc_library(
    name = "sdc",
    srcs = glob([
        "opensubdiv/sdc/**/*.cpp",
        "opensubdiv/sdc/**/*.c",
    ]) + ["opensubdiv/version.h"],
    hdrs = glob([
        "opensubdiv/sdc/**/*.hpp",
        "opensubdiv/sdc/**/*.h",
    ]),
    copts = [],
    defines = [],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "hbr",
    srcs = glob([
        "opensubdiv/hbr/**/*.cpp",
        "opensubdiv/hbr/**/*.c",
    ]) + ["opensubdiv/version.h"],
    hdrs = glob([
        "opensubdiv/hbr/**/*.hpp",
        "opensubdiv/hbr/**/*.h",
    ]),
    copts = [],
    defines = [],
    visibility = ["//visibility:public"],
    deps = [],
)
