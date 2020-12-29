"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "state",
    hdrs = [
        "state.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "io",
    hdrs = ["io.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":state",
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "//messages:state_proto",
        "//messages:tensor",
    ],
)

isaac_cc_library(
    name = "state_utils",
    hdrs = [
        "state_utils.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":state",
        "@com_nvidia_isaac_engine//engine/core/tensor",
    ],
)
