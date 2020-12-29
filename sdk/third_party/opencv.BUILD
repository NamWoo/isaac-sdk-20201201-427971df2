"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "opencv_x86_64",
    srcs = [":so"],
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
    ]),
    data = [":so"],
    linkopts = [
        "-Wl,--no-as-needed," +
        "-l:libopencv_features2d.so.4.1," +
        "-l:libopencv_flann.so.4.1," +
        "-l:libopencv_imgproc.so.4.1," +
        "--as-needed",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

filegroup(
    name = "so",
    srcs = glob(
        ["lib/*.so*"],
        exclude = ["lib/libtbb*"],
    ),
)
