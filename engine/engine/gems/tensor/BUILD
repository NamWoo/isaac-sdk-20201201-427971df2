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
    name = "tensor",
    visibility = ["//visibility:public"],
    deps = [
        "transpose",
        "utils",
    ],
)

isaac_cc_library(
    name = "transpose",
    srcs = ["transpose.cpp"],
    hdrs = ["transpose.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "transpose_cuda",
        "utils",
        "//engine/core",
        "//engine/core/tensor",
        "//third_party/nlohmann",
    ],
)

cc_cuda_library(
    name = "transpose_cuda",
    srcs = [
        "transpose_201.cu.cpp",
    ],
    hdrs = [
        "transpose_201.cu.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/cuda_utils",
        "//third_party:cudart",
    ],
)

isaac_cc_library(
    name = "utils",
    hdrs = ["utils.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/tensor",
        "//engine/gems/math",
    ],
)
