"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_pkg")
load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "engine_c_api",
    srcs = [
        "application_c_api.cpp",
        "application_c_api.hpp",
        "isaac_c_api.cpp",
        "isaac_c_api_error.cpp",
        "message_c_api.cpp",
        "message_c_api.hpp",
    ],
    hdrs = [
        "isaac_c_api.h",
        "isaac_c_api_error.h",
        "isaac_c_api_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//messages",
        "//messages:camera_proto",
        "//messages:proto_registry",
        "//messages/utils",
        "@com_nvidia_isaac_engine//engine/alice",
        "@com_nvidia_isaac_engine//engine/core",
    ],
    alwayslink = True,
)

isaac_cc_binary(
    name = "libisaac_c_api.so",
    linkshared = True,
    visibility = ["//visibility:public"],
    deps = [":engine_c_api"],
)

filegroup(
    name = "public_headers",
    data = [
        "isaac_c_api.h",
        "isaac_c_api_error.h",
        "isaac_c_api_types.h",
    ],
)

isaac_pkg(
    name = "isaac_engine_c_api-pkg",
    srcs = [
        "libisaac_c_api.so",
        "public_headers",
        "//packages/engine_tcp_udp:libengine_tcp_udp_module.so",
        "//packages/sight:libsight_module.so",
    ],
)
