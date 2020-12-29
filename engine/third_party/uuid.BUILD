"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

package(default_visibility = ["//visibility:public"])

licenses(["notice"])  # GNU GENERAL PUBLIC LICENSE

cc_library(
    name = "libuuid",
    srcs = glob(
        [
            "*.c",
            "*.h",
        ],
        exclude = ["test_uuid.c"],
    ),
    hdrs = ["uuid.h"],
    defines = [
        "HAVE_USLEEP",
        "HAVE_SYS_FILE_H",
        "HAVE_STDLIB_H",
        "HAVE_UNISTD_H",
        "HAVE_SYS_SOCKET_H",
    ],
    includes = [
        ".",
    ],
)
