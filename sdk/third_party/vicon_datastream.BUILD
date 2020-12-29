"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

licenses(["notice"])  # MIT License

cc_library(
    name = "vicon_datastream",
    srcs = glob(
        [
            "vicon-datastream-sdk/libViconDataStreamSDK_CPP.so",
            "vicon-datastream-sdk/*.so.1.53.0",
        ],
        exclude = ["vicon-datastream-sdk/libboost_python*"],
    ),
    hdrs = glob([
        "vicon-datastream-sdk/DataStreamClient.h",
    ]),
    linkopts = ["-lViconDataStreamSDK_CPP"],
    strip_include_prefix = "vicon-datastream-sdk",
    visibility = ["//visibility:public"],
)
