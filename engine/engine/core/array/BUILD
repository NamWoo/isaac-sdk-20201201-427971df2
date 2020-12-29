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
    name = "array",
    hdrs = ["byte_array.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "cpu_array",
        "cuda_array",
    ],
)

isaac_cc_library(
    name = "cpu_array",
    hdrs = [
        "cpu_array.hpp",
        "cpu_array_view.hpp",
    ],
    deps = [
        "//engine/core",
        "//engine/core/buffers",
    ],
)

isaac_cc_library(
    name = "cuda_array",
    srcs = ["cuda_array.cpp"],
    hdrs = [
        "cuda_array.hpp",
        "cuda_array_view.hpp",
    ],
    deps = [
        "cpu_array",
        "//engine/core",
        "//engine/core/buffers",
        "//third_party:cudart",
    ],
)
