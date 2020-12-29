"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

filegroup(
    name = "sphinx_files",
    srcs = [
        "_static/css/isaac_custom.css",
        "_static/logo_isaac.png",
        "conf.py",
        "nvidia.png",
    ],
    visibility = ["//visibility:public"],
)

sh_binary(
    name = "sphinx_latexpdf",
    srcs = ["sphinx_latexpdf.sh"],
    data = [":sphinx_files"],
    visibility = ["//visibility:public"],
)

sh_binary(
    name = "sphinx_html",
    srcs = ["sphinx_html.sh"],
    data = [":sphinx_files"],
    visibility = ["//visibility:public"],
)
