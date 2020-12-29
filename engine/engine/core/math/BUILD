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
    name = "math",
    hdrs = [
        "pose2.hpp",
        "pose3.hpp",
        "quaternion.hpp",
        "so2.hpp",
        "so3.hpp",
        "types.hpp",
        "utils.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "@org_tuxfamily_eigen//:eigen",
    ],
)

isaac_cc_library(
    name = "dual_quaternion",
    hdrs = ["dual_quaternion.hpp"],
    srcs = ["dual_quaternion.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":math",
        "//engine/core",
    ],
)
