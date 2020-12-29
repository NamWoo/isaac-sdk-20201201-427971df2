"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:cc_cuda_library.bzl", "cc_cuda_library")
load("//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "core_dump",
    srcs = ["core_dump.cpp"],
    hdrs = ["core_dump.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/core"],
)

cc_cuda_library(
    name = "cuda_context",
    srcs = ["cuda_context.cpp"],
    hdrs = ["cuda_context.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/core"],
)

isaac_cc_library(
    name = "filesystem",
    srcs = ["filesystem.cpp"],
    hdrs = ["filesystem.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/core"],
)

isaac_cc_library(
    name = "network",
    srcs = ["network.cpp"],
    hdrs = ["network.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/core"],
)
