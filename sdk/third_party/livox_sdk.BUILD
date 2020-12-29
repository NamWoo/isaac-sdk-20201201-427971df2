"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "livox_sdk",
    srcs = [
        "sdk_core/src/third_party/FastCRC/FastCRCsw.cpp",
    ],
    hdrs = [
        "sdk_core/include/comm/protocol.h",
        "sdk_core/include/livox_def.h",
        "sdk_core/include/third_party/FastCRC/FastCRC.h",
        "sdk_core/src/comm/sdk_protocol.h",
        "sdk_core/src/command_handler/command_impl.h",
        "sdk_core/src/third_party/FastCRC/FastCRC_tables.hpp",
    ],
    includes = ["sdk_core/include"],
    visibility = ["//visibility:public"],
)
