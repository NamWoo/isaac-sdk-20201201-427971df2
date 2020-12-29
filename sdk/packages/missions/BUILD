"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//doc:doc.bzl", "isaac_doc_dep")

py_library(
    name = "missions",
    srcs = [
        "JsonTcpServer.py",
        "MissionServer.py",
        "__init__.py",
    ],
    visibility = ["//visibility:public"],
)

isaac_doc_dep(
    name = "doc",
    srcs = [
        "doc/images/app_with_missions.png",
        "doc/images/mission_dep.png",
        "doc/missions.rst",
    ],
    visibility = ["//visibility:public"],
)
