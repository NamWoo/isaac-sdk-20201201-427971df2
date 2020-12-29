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
    name = "interpolation",
    hdrs = ["interpolation.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":cubic",
        ":linear",
        ":poses",
    ],
)

isaac_cc_library(
    name = "linear",
    hdrs = [
        "bilinear_approximated_function.hpp",
        "linear.hpp",
        "linear_approximated_function.hpp",
        "utils.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/tensor",
        "//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "cubic",
    hdrs = [
        "bicubic_approximated_function.hpp",
        "cubic.hpp",
        "cubic_approximated_function.hpp",
        "utils.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/tensor",
        "//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "poses",
    hdrs = ["poses.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":linear",
        "//engine/gems/math",
    ],
)
