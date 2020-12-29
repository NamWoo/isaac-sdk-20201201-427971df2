"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
# Description:
#   Apriltag is a library to detect fiducials

cc_library(
    name = "apriltags3",
    srcs = glob([
        "*.c",
        "*.h",
        "common/*.c",
        "common/*.h",
        ],
        exclude = ["apriltag_pywrap.c"],
    ),
    hdrs = ["apriltag.h"],
    defines = ["HAVE_STD_REGEX"],
    copts = [
        "-Wno-unused-but-set-variable",
        "-Wno-unused-variable",
    ],
    includes = ["include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)