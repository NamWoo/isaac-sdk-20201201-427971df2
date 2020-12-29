"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
# Description:
#   Realsense sdk

licenses(["notice"])  # Apache 2.0. See LICENSE file

exports_files([
    "LICENSE",
    "COPYING",
])

cc_library(
    name = "realsense",
    srcs = glob(
        [
            "src/**/*.cpp",
            "src/**/*.h",
            "src/**/*.hpp",
        ],
        exclude = [
            "src/android/*",
            "src/android/jni/*",
            "src/android/usb_host/*",
            "src/cuda/*",
            "src/fw/*",
            "src/gl/*",
            "src/libuvc/*",
            "src/mf/*",
            "src/tm2/*",
            "src/win/*",
            "src/win7/*",
            "src/winusb/*",
        ],
    ) + [
        "common/fw/firmware-version.h",
        "src/libuvc/uvc_types.h",
        "third-party/stb_image.h",
    ],
    hdrs = glob([
        "include/librealsense2/**/*.h*",
    ]),
    copts = [
        # When preprocessing, do not shorten system header paths with canonicalization.
        "-fno-canonical-system-headers",
        # Disable all warnings.
        # librealsense2 produces a large number of warnings. Not all can blocked with -Wno- flags.
        # So we need to use the heavy handed approach of disabling all warnings.
        "-w",
        "-Wno-error",
    ],
    defines = [
        "RS2_USE_V4L2_BACKEND",
        "HWM_OVER_XU",
    ],
    includes = [
        "include",
        "src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":json",
        ":realsense-file",
        ":sqlite",
        "@libusb",
    ],
)

cc_library(
    name = "easylogging",
    srcs = ["third-party/easyloggingpp/src/easylogging++.cc"],
    hdrs = ["third-party/easyloggingpp/src/easylogging++.h"],
    includes = ["third-party/easyloggingpp/src"],
)

cc_library(
    name = "realsense-file",
    srcs = glob([
        "third-party/realsense-file/lz4/lz4.c",
        "third-party/realsense-file/rosbag/**/*.c",
        "third-party/realsense-file/rosbag/**/*.cpp",
    ]),
    hdrs = glob([
        "third-party/realsense-file/**/*.h",
        "third-party/realsense-file/**/*.hpp",
    ]),
    copts = [
        # When preprocessing, do not shorten system header paths with canonicalization.
        "-fno-canonical-system-headers",
        "-Wno-unused-variable",
        "-Wno-misleading-indentation",
        "-Wno-deprecated",
    ],
    includes = [
        "third-party/realsense-file/boost",
        "third-party/realsense-file/lz4",
        "third-party/realsense-file/rosbag/console_bridge/include",
        "third-party/realsense-file/rosbag/cpp_common/include",
        "third-party/realsense-file/rosbag/msgs",
        "third-party/realsense-file/rosbag/rosbag_storage/include",
        "third-party/realsense-file/rosbag/roscpp_serialization/include",
        "third-party/realsense-file/rosbag/roscpp_traits/include",
        "third-party/realsense-file/rosbag/roslz4/include",
        "third-party/realsense-file/rosbag/rostime/include",
    ],
)

cc_library(
    name = "sqlite",
    srcs = ["third-party/sqlite/sqlite3.c"],
    hdrs = ["third-party/sqlite/sqlite3.h"],
)

cc_library(
    name = "json",
    hdrs = ["third-party/json.hpp"],
)
