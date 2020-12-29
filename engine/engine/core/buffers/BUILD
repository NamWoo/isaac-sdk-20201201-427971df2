"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "algorithm",
    srcs = select({
        "//conditions:default": ["algorithm_cuda.cpp"],
        "//engine/build:isaac_gpu_disabled": ["algorithm_cpu.cpp"],
    }),
    hdrs = ["algorithm.hpp"],
    deps = [
        ":utils",
        "//engine/core",
    ] + select({
        "//conditions:default": ["//third_party:cudart"],
        "//engine/build:isaac_gpu_disabled": [],
    }),
)

isaac_cc_library(
    name = "buffers",
    visibility = ["//visibility:public"],
    deps = [
        ":algorithm",
        ":buffer",
        ":shared_buffer",
        ":utils",
    ],
)

isaac_cc_library(
    name = "buffer",
    srcs = ["buffer.cpp"],
    hdrs = ["buffer.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":utils",
        "//engine/core",
        "//engine/core/allocator",
    ],
)

isaac_cc_library(
    name = "shared_buffer",
    srcs = ["shared_buffer.cpp"],
    hdrs = ["shared_buffer.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":algorithm",
        ":buffer",
        ":utils",
    ],
)

isaac_cc_library(
    name = "utils",
    hdrs = ["traits.hpp"],
)
