"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "opencv_aarch64_jetpack44",
    srcs = [":so"],
    hdrs = glob([
        "usr/include/**/*.h",
        "usr/include/**/*.hpp",
    ]),
    data = [":so"],
    linkopts = [
        "-Wl,--no-as-needed," +
        "-l:libopencv_features2d.so.4.1," +
        "-l:libopencv_flann.so.4.1," +
        "-l:libopencv_imgproc.so.4.1," +
        "--as-needed",
    ],
    strip_include_prefix = "usr/include/opencv4",
    visibility = ["//visibility:public"],
)

filegroup(
    name = "so",
    srcs = glob(["usr/lib/aarch64-linux-gnu/*.so*"]),
)
