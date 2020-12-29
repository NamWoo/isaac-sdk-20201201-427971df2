"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_pkg")

py_binary(
    name = "record_dummy",
    srcs = ["record_dummy.py"],
    data = [
        "__init__.py",
        "//apps:py_init",
        "//apps/samples:py_init",
        "//packages/cask:libcask_module.so",
        "//packages/message_generators:libmessage_generators_module.so",
        "//packages/sight:libsight_module.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/pyalice",
    ],
)

py_binary(
    name = "record_realsense",
    srcs = ["record_realsense.py"],
    data = [
        "__init__.py",
        "//apps:py_init",
        "//apps/samples:py_init",
        "//packages/cask:libcask_module.so",
        "//packages/realsense:librealsense_module.so",
        "//packages/sight:libsight_module.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/pyalice",
    ],
)

isaac_pkg(
    name = "record-pkg",
    srcs = [
        "record_dummy",
        "record_realsense",
    ],
)
