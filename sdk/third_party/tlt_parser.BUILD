"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "tlt_parser",
    linkstatic = True,
    visibility = ["//visibility:public"],
    deps = select({
        "@com_nvidia_isaac_engine//engine/build:platform_x86_64": ["tlt_parser_x86_64"],
        "@com_nvidia_isaac_engine//engine/build:platform_jetpack44": ["tlt_parser_aarch64_jetpack43"],
    }),
)

cc_library(
    name = "tlt_parser_x86_64",
    srcs = [
        "src/libtlt_parser_x86_64.a",
    ],
    hdrs = [
        "include/tlt_parser.h",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "tlt_parser_aarch64_jetpack43",
    srcs = [
        "src/libtlt_parser_jetpack43.a",
    ],
    hdrs = [
        "include/tlt_parser.h",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
