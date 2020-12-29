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
    name = "serialization",
    srcs = [
        "blob.cpp",
        "header.cpp",
    ],
    hdrs = [
        "blob.hpp",
        "header.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/array",
        "//engine/core/buffers",
        "//engine/gems/uuid",
        "@capnproto//:capnp_lite",
    ],
)

isaac_cc_library(
    name = "json",
    srcs = [
        "json.cpp",
    ],
    hdrs = [
        "json.hpp",
        "json_formatter.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/image",
        "//engine/core/math",
        "//engine/gems/geometry",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "files",
    srcs = [
        "files.cpp",
    ],
    hdrs = [
        "files.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
    ],
)

isaac_cc_library(
    name = "crc16",
    hdrs = [
        "bytes_convert.hpp",
        "crc16.hpp",
    ],
    visibility = ["//visibility:public"],
)

isaac_cc_library(
    name = "base64",
    srcs = ["base64.cpp"],
    hdrs = ["base64.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/gems/image"],
)

isaac_cc_library(
    name = "capnp",
    srcs = [
        "capnp.cpp",
    ],
    hdrs = [
        "capnp.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/buffers",
        "@capnproto//:capnp_lite",
    ],
)
