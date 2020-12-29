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
    name = "element_type",
    hdrs = ["element_type.hpp"],
    srcs = ["element_type.cpp"],
    visibility = ["//visibility:public"],
)

isaac_cc_library(
    name = "tensor",
    hdrs = ["tensor.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/array",
        "//engine/core/buffers",
        "//engine/core/buffers:buffer",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "sample_cloud",
    hdrs = ["sample_cloud.hpp"],
    visibility = ["//visibility:public"],
    deps = ["tensor"],
)

isaac_cc_library(
    name = "universal_tensor",
    hdrs = ["universal_tensor.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "element_type",
        "tensor",
        "//engine/core",
        "//engine/core/buffers",
        "//engine/core/math",
    ],
)
