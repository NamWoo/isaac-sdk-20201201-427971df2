"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "common",
    hdrs = ["domains.hpp"],
    deps = ["//packages/engine_gems/state"],
)

cc_test(
    name = "io",
    size = "small",
    srcs = ["io.cpp"],
    deps = [
        ":common",
        "//packages/engine_gems/state",
        "//packages/engine_gems/state:io",
        "@gtest//:main",
    ],
)

cc_test(
    name = "state",
    size = "small",
    srcs = ["state.cpp"],
    deps = [
        ":common",
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/engine_gems/state",
        "@gtest//:main",
    ],
)
