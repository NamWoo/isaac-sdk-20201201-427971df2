"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "dynamixel_include_hdr",
    hdrs = glob([
        "c++/include/dynamixel_sdk/dynamixel_sdk.h",
        "c++/include/dynamixel_sdk/group*.h",
        "c++/include/dynamixel_sdk/packet*.h",
        "c++/include/dynamixel_sdk/port_handler.h",
        "c++/include/dynamixel_sdk/port_handler_linux.h",
        "c++/include/dynamixel_sdk/protocol1_packet_handler.h",
        "c++/include/dynamixel_sdk/protocol2_packet_handler.h",
    ]),
    strip_include_prefix = "c++/include/dynamixel_sdk",
)

cc_library(
    name = "dynamixel",
    srcs = glob([
        "c++/src/dynamixel_sdk/group*.cpp",
        "c++/src/dynamixel_sdk/packet_handler.cpp",
        "c++/src/dynamixel_sdk/port_handler.cpp",
        "c++/src/dynamixel_sdk/port_handler_linux.cpp",
        "c++/src/dynamixel_sdk/protocol1_packet_handler.cpp",
        "c++/src/dynamixel_sdk/protocol2_packet_handler.cpp",
    ]),
    hdrs = glob([
        "c++/src/*.h",
    ]),
    copts = [
        "-D_GNU_SOURCE",
        "-DLINUX",
        "-Wall",
    ],
    defines = [
        "LINUX",
        "_GNU_SOURCE",
    ],
    strip_include_prefix = "c++/src",
    visibility = ["//visibility:public"],
    deps = [
        ":dynamixel_include_hdr",
    ],
)

# This is a test app provided by Robotis that can be used as a standalone binary
cc_binary(
    name = "dynamixel_manager",
    srcs = [
        "c++/example/dxl_monitor/dxl_monitor.cpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@robotis//:dynamixel",
    ],
)
