"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

py_binary(
    name = "gl3w_gen",
    srcs = [
        "gl3w_gen.py",
    ],
    main = "gl3w_gen.py",
    visibility = ["//visibility:public"],
)

_GL3W_SRCS = ["src/gl3w.c"]

_GL3W_HDRS = [
    "include/GL/gl3w.h",
    "include/GL/glcorearb.h",
    "include/KHR/khrplatform.h",
]

genrule(
    name = "gl3w_genrule",
    srcs = glob(["**/*"]),
    outs = _GL3W_SRCS + _GL3W_HDRS,
    cmd = "python $(location gl3w_gen.py) --root $(@D)",
)

cc_library(
    name = "gl3w",
    srcs = _GL3W_SRCS,
    hdrs = _GL3W_HDRS,
    includes = ["include"],
    visibility = ["//visibility:public"],
)
