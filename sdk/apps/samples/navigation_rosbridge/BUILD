"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "navigation_ros_bridge",
    srcs = [
        "NavigationRosBridge.cpp",
    ],
    hdrs =
        ["NavigationRosBridge.hpp"],
    tags = [
        "manual",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//packages/ros_bridge/gems",
        "//third_party:ros",
        "@com_nvidia_isaac_engine//engine/core/image",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/gems/sight",
    ],
)

isaac_app(
    name = "navigation_rosbridge",
    app_json_file = "navigation_rosbridge.app.json",
    modules = [
        "//apps/samples/navigation_rosbridge:navigation_ros_bridge",
        "sight",
    ],
    tags = ["manual"],
)
