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
    name = "state",
    visibility = ["//visibility:public"],
    deps = [
        ":differential_base",
        ":holonomic_base",
    ],
)

isaac_cc_library(
    name = "audio",
    hdrs = ["audio.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/engine_gems/state",
        "//packages/composite/gems:typed_composite_view",
    ],
)

isaac_cc_library(
    name = "common",
    hdrs = ["common.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/engine_gems/state",
        "//packages/composite/gems:typed_composite_view",
    ],
)

isaac_cc_library(
    name = "ackermann_base",
    hdrs = ["ackermann_base.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/engine_gems/state",
        "//packages/composite/gems:typed_composite_view",
    ],
)

isaac_cc_library(
    name = "differential_base",
    hdrs = ["differential_base.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/engine_gems/state",
        "//packages/composite/gems:typed_composite_view",
    ],
)

isaac_cc_library(
    name = "holonomic_base",
    hdrs = ["holonomic_base.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/engine_gems/state",
        "//packages/composite/gems:typed_composite_view",
    ],
)

isaac_cc_library(
    name = "pan_tilt",
    hdrs = ["pan_tilt.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/engine_gems/state",
        "//packages/composite/gems:typed_composite_view",
    ],
)
