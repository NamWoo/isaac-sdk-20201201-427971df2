"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
# Description:
#   Eigen is a C++ template library for linear algebra: vectors,
#   matrices, and related algorithms.

cc_library(
    name = "eigen",
    srcs = glob(["Eigen/src/**/*.h"]),
    hdrs = glob(["Eigen/*"]),
    copts = ["-DEIGEN_MPL2_ONLY"],
    includes = ["."],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "unsupported",
    srcs = glob(
        ["unsupported/Eigen/src/**/*"],
        exclude = ["**/*.txt"],
    ),
    hdrs = glob(["unsupported/Eigen/*"]),
    copts = ["-DEIGEN_MPL2_ONLY"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
