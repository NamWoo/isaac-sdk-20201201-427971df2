"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary")
load("@com_nvidia_isaac_engine//engine/build:cc_capnp_library.bzl", "cc_capnp_library")

cc_capnp_library(
    name = "protos",
    protos = glob(["*.capnp"]),
    deps = ["//messages:uuid"],
)

isaac_cc_binary(
    name = "header",
    srcs = ["header.cpp"],
    deps = [
        ":protos",
        "@com_nvidia_isaac_engine//engine/gems/serialization",
        "//messages:uuid",
        "@benchmark",
        "@capnproto//:capnp_lite",
    ],
)

isaac_cc_binary(
    name = "base64",
    srcs = ["base64.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/serialization:base64",
        "@benchmark",
    ],
)
