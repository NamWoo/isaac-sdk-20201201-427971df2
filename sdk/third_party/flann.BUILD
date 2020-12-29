"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
# Description:
#   flann is a dependency for PCL

licenses(["notice"])  # BSD/MIT-like license

exports_files(["COPYING"])

cc_library(
    name = "flann",
    srcs = glob(
        [
            "src/cpp/flann/**/*.c",
            "src/cpp/flann/**/*.cpp",
        ],
        exclude = [
            "src/cpp/flann/mpi/*.cpp",
            "src/cpp/flann/ext/*.c",
        ],
    ),
    hdrs = glob(
        [
            "src/cpp/flann/**/*.h",
            "src/cpp/flann/**/*.h.in",
            "src/cpp/flann/**/*.hpp",
        ],
        exclude = [
            "src/cpp/flann/io/hdf5.h",
            "src/cpp/flann/ext/*.h",
        ],
    ),
    copts = [
        "-fopenmp",
        "-Wno-misleading-indentation",
    ],
    strip_include_prefix = "src/cpp/",
    visibility = ["//visibility:public"],
    deps = [":lz4"],
)

cc_library(
    name = "lz4",
    srcs = glob(["src/cpp/flann/ext/*.c"]),
    hdrs = glob(["src/cpp/flann/ext/*.h"]) + [
        "src/cpp/flann/ext/lz4.c",
    ],
    copts = [
        "-fopenmp",
        "-Wno-misleading-indentation",
    ],
    strip_include_prefix = "src/cpp/flann/ext",
)
