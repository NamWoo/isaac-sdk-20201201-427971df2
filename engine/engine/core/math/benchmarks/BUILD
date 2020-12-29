"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "helpers",
    hdrs = ["cache.hpp"],
)

isaac_cc_binary(
    name = "floor",
    srcs = ["floor.cpp"],
    deps = [
        ":helpers",
        "//engine/core/math",
        "@benchmark",
    ],
)

isaac_cc_binary(
    name = "modulo",
    srcs = ["modulo.cpp"],
    deps = [
        ":helpers",
        "//engine/core/math",
        "@benchmark",
    ],
)
